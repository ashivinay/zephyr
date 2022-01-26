import gdb

i = 0
while True:
    for j in range(15):
        gdb.write("%d" % i)
        gdb.execute("p gpt_trace_events[gpt_trace_idx - %d]" % (i))
        i+=1
    if (input("Continue? [Y/N]") != 'Y'):
        break
