import gdb
import struct
import os

# GPT trace types (should match enum of trace types in C source)
GPT_TRACE_TYPES = [
    "MCUX_GPT_ISR",
    "SYS_CLOCK_SET_TIMEOUT",
    "SYS_CLOCK_ELAPSED",
    "SYS_CLOCK_CYCLES_GET_32",
    "EXTERNAL"
]

# Struct format strings, see https://docs.python.org/3/library/struct.html#format-strings
GPT_HEADER_FORMAT = "<10sc6I"
GPT_TRACE_FORMAT = "<7I"

# Required structure definitions in GPT source:

# struct gpt_trace_record {
#   uint32_t trace_id;
# 	uint32_t idx;
# 	uint32_t now;
# 	uint32_t announced;
# 	uint32_t requested;
# 	uint32_t new_load;
#   uint32_t rollover;
# } __packed;
#
# struct gpt_trace_data {
# 	char header[10];
#   bool attached;
# 	uint32_t trace_buf_size;
# 	uint32_t trace_idx;
# 	uint32_t cycle_per_tick;
# 	uint32_t max_cycle;
# 	uint32_t min_delay;
# 	struct gpt_trace_record data[TRACE_SIZE];
# } __packed;

"""Writes GPT trace entry from buffer at offset"""
def write_entry(f, buf, off):
    gpt_trace = struct.unpack_from(GPT_TRACE_FORMAT, buf,
                                offset=off * struct.calcsize(GPT_TRACE_FORMAT))
    f.write(f"{gpt_trace[0]},{GPT_TRACE_TYPES[gpt_trace[1]]},{gpt_trace[2]},")
    f.write(f"{gpt_trace[3]},{gpt_trace[4]},{gpt_trace[5]},{gpt_trace[6]}\n")

class GPTTraceBP(gdb.Breakpoint):
    def __init__(self, trace_addr, filename):
        super(GPTTraceBP, self).__init__("_gpt_break_sym")
        self.trace_addr = trace_addr
        self.target = gdb.selected_inferior()
        self.filename = filename

    """Collect GPT trace data from system into CSV file"""
    def stop(self):
        # Dump GPT trace data to memory
        header_mem = self.target.read_memory(self.trace_addr, struct.calcsize(GPT_HEADER_FORMAT))
        gpt_header = struct.unpack(GPT_HEADER_FORMAT, header_mem)

        trace_size = gpt_header[2]
        cycle_per_tick = gpt_header[4]
        max_cycle = gpt_header[5]
        min_delay = gpt_header[6]
        # Trace data is at end of GPT trace structure
        trace_data_addr = self.trace_addr + (struct.calcsize(GPT_HEADER_FORMAT) - 4)

        print(f"Cycle per tick: {cycle_per_tick}, Max cycle: {max_cycle}, Min delay: {min_delay}")
        print(f"Reading traces from 0x{trace_data_addr:x}")
        size = struct.calcsize(GPT_TRACE_FORMAT) * trace_size
        print("Reading %d bytes" % (size))

        # Read and dump GPT trace
        gpt_trace_buf = self.target.read_memory(trace_data_addr, size)

        with open(self.filename, "a") as f:
            # Take slices of GPT trace, and dump to CSV
            for i in range(trace_size):
                write_entry(f, gpt_trace_buf, i)
            f.close()
        # Clear R0 to get out of the loop the code will be in
        gdb.execute("set $r0=1")
        return False

class GPTInitBP(gdb.Breakpoint):
    def __init__(self, filename):
        super(GPTInitBP, self).__init__("z_cstart")
        self.filename = filename
        self.breakpoint_installed = False
        self.trace_addr = None
        # Get target device
        self.target = gdb.selected_inferior()
        # Overwrite file if it exists
        with open(self.filename, "w") as f:
            f.write("Idx, type, GPT count, announced, requested, new_load, rollover\n")
            f.close()

    """Gets GPT trace buffer address at reset"""
    def stop(self):

        if self.trace_addr is None:
            self.attach()
        # Setup GPT trace breakpoint
        if not self.breakpoint_installed:
            self.trace_bp = GPTTraceBP(self.trace_addr, self.filename)
            self.breakpoint_installed = True
        return False

    """Attaches to running program, reading the trace buffer address"""
    def attach(self):
        # Find GPT trace in memory (assumed trace is in DTCM)
        self.trace_addr = self.target.search_memory(0x20000000, 0x20000, "GPT_TRACE")
        print(f"Found GPT trace at 0x{self.trace_addr:x}")
        if self.trace_addr is None:
            raise gdb.GdbError("Error, could not find GPT trace?")
        # Mark the "attached" field of the struct as true to start tracing
        header_mem = self.target.read_memory(self.trace_addr, struct.calcsize(GPT_HEADER_FORMAT))
        gpt_header = list(struct.unpack(GPT_HEADER_FORMAT, header_mem))
        gpt_header[1] = b'\x01'
        self.target.write_memory(self.trace_addr, struct.pack(GPT_HEADER_FORMAT,
            *gpt_header))
        if not self.breakpoint_installed:
            self.trace_bp = GPTTraceBP(self.trace_addr, self.filename)
            self.breakpoint_installed = True


    """Flushes trace data, and resets trace buffer index"""
    def flush_trace(self):
        if self.trace_addr is None:
            self.attach()
        # Dump GPT trace data to memory
        header_mem = self.target.read_memory(self.trace_addr, struct.calcsize(GPT_HEADER_FORMAT))
        gpt_header = list(struct.unpack(GPT_HEADER_FORMAT, header_mem))

        trace_size = gpt_header[2]
        # Get offset to current trace in GPT buffer
        trace_offset = (gpt_header[3]) % trace_size
        cycle_per_tick = gpt_header[4]
        max_cycle = gpt_header[5]
        min_delay = gpt_header[6]
        # Trace data is at end of GPT trace structure
        trace_data_addr = self.trace_addr + (struct.calcsize(GPT_HEADER_FORMAT) - 4)

        print(f"Cycle per tick: {cycle_per_tick}, Max cycle: {max_cycle}, Min delay: {min_delay}")
        print(f"Trace offset is currently {trace_offset - 1}")
        print(f"Reading traces from 0x{trace_data_addr:x}")
        size = struct.calcsize(GPT_TRACE_FORMAT) * trace_offset
        print("Reading %d bytes" % (size))

        # Read and dump GPT trace
        gpt_trace_buf = self.target.read_memory(trace_data_addr, size)

        with open(self.filename, "a") as f:
            # Take slices of GPT trace, and dump to CSV
            for i in range(0, trace_offset):
                write_entry(f, gpt_trace_buf, i)
            f.close()
        # Reset trace buffer to first offset
        gpt_header[3] = 0
        # The * here "splats" the gpt header tuple as arguments
        self.target.write_memory(self.trace_addr,
            struct.pack(GPT_HEADER_FORMAT, *gpt_header))


    """Disables trace breakpoint, and removes this breakpoint"""
    def disable_trace(self):
        # Mark the "attached" field of the struct as false to stop tracking
        header_mem = self.target.read_memory(self.trace_addr, struct.calcsize(GPT_HEADER_FORMAT))
        gpt_header = list(struct.unpack(GPT_HEADER_FORMAT, header_mem))
        gpt_header[1] = b'\x00'
        self.target.write_memory(self.trace_addr, struct.pack(GPT_HEADER_FORMAT,
            *gpt_header))
        self.trace_bp.delete()
        self.delete()


class GPTCommand(gdb.Command):
    """Manages GPT trace dump"""
    def __init__(self):
        super(GPTCommand, self).__init__("gpttrace",
            gdb.COMMAND_USER, completer_class=gdb.COMPLETE_FILENAME)

    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        if len(args) == 2:
            if 'en' in args[0]:
                # Enable trace breakpoints
                self.filename = args[1]
                self.init_bp = GPTInitBP(self.filename)
                print("Tracing enabled")
                return
            if 'at' in args[0]:
                # Enable trace breakpoints
                self.filename = args[1]
                self.init_bp = GPTInitBP(self.filename)
                self.init_bp.attach()
                print("Tracing attached")
                return
        elif len(args) == 1:
            if 'fl' in args[0]:
                # Flush the remaining data
                print(f"Flushing to {self.filename}")
                self.init_bp.flush_trace()
                return
            elif 'dis' in args[0]:
                self.init_bp.disable_trace()
                print("Tracing disabled")
                return
        # If we get here, argument was incorrect
        raise gdb.GdbError("Error, argument must either be flush/disable, or enable/attach with a filename")
# Init GPT command instance
GPTCommand()
