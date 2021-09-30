/**
 * Copyright (c) 2021, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <fsl_os_abstraction.h>
#include <kernel.h>
#include <irq.h>

static unsigned int gIrqKey;

/*!
 * @brief Reserves the requested amount of memory in bytes.
 *
 * The function is used to reserve the requested amount of memory in bytes and initializes it to 0.
 *
 * @param length Amount of bytes to reserve.
 *
 * @return Pointer to the reserved memory. NULL if memory can't be allocated.
 */
void *OSA_MemoryAllocate(uint32_t length)
{
	return k_malloc(length);
}

/*!
 * @brief Frees the memory previously reserved.
 *
 * The function is used to free the memory block previously reserved.
 *
 * @param p Pointer to the start of the memory block previously reserved.
 *
 */
void OSA_MemoryFree(void *p)
{
	return k_free(p);
}

/*!
 * @brief Enter critical with nesting mode.
 *
 * @param sr Store current status and return to caller.
 */
void OSA_EnterCritical(uint32_t *sr)
{
	*sr = irq_lock();
}

/*!
 * @brief Exit critical with nesting mode.
 *
 * @param sr Previous status to restore.
 */
void OSA_ExitCritical(uint32_t sr)
{
	irq_unlock(sr);
}


/*!
 * @brief Creates a semaphore with a given value.
 *
 * This function creates a semaphore and sets the value to the parameter
 * initValue.
 *
 * Example below shows how to use this API to create the semaphore handle.
 * @code
 *   OSA_SEMAPHORE_HANDLE_DEFINE(semaphoreHandle);
 *   OSA_SemaphoreCreate((osa_semaphore_handle_t)semaphoreHandle, 0xff);
 * @endcode
 *
 * @param semaphoreHandle Pointer to a memory space of size OSA_SEM_HANDLE_SIZE
 * allocated by the caller. The handle should be 4 byte aligned, because unaligned access
 * doesn't be supported on some devices.
 * You can define the handle in the following two ways:
 * #OSA_SEMAPHORE_HANDLE_DEFINE(semaphoreHandle);
 * or
 * uint32_t semaphoreHandle[((OSA_SEM_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
 * @param initValue Initial value the semaphore will be set to.
 *
 * @retval KOSA_StatusSuccess  the new semaphore if the semaphore is created successfully.
 * @retval KOSA_StatusError   if the semaphore can not be created.
 */
osa_status_t OSA_SemaphoreCreate(osa_semaphore_handle_t semaphoreHandle, uint32_t initValue)
{
	struct k_sem *sem = (struct k_sem *)semaphoreHandle;

	if (k_sem_init(sem, initValue, K_SEM_MAX_LIMIT)) {
		return KOSA_StatusError;
	}
	return KOSA_StatusSuccess;
}

/*!
 * @brief Destroys a previously created semaphore.
 *
 * @param semaphoreHandle The semaphore handle.
 * The macro SEMAPHORE_HANDLE_BUFFER_GET is used to get the semaphore buffer pointer,
 * and should not be used before the macro SEMAPHORE_HANDLE_BUFFER_DEFINE is used.
 *
 * @retval KOSA_StatusSuccess The semaphore is successfully destroyed.
 * @retval KOSA_StatusError   The semaphore can not be destroyed.
 */
osa_status_t OSA_SemaphoreDestroy(osa_semaphore_handle_t semaphoreHandle)
{
	struct k_sem *sem = (struct k_sem *)semaphoreHandle;

	k_sem_reset(sem);
	return KOSA_StatusSuccess;
}

/*!
 * @brief Pending a semaphore with timeout.
 *
 * This function checks the semaphore's counting value. If it is positive,
 * decreases it and returns KOSA_StatusSuccess. Otherwise, a timeout is used
 * to wait.
 *
 * @param semaphoreHandle    The semaphore handle.
 * @param millisec The maximum number of milliseconds to wait if semaphore is not
 *                 positive. Pass osaWaitForever_c to wait indefinitely, pass 0
 *                 will return KOSA_StatusTimeout immediately.
 *
 * @retval KOSA_StatusSuccess  The semaphore is received.
 * @retval KOSA_StatusTimeout  The semaphore is not received within the specified 'timeout'.
 * @retval KOSA_StatusError    An incorrect parameter was passed.
 */
osa_status_t OSA_SemaphoreWait(osa_semaphore_handle_t semaphoreHandle, uint32_t millisec)
{
	struct k_sem *sem = (struct k_sem *)semaphoreHandle;
	int ret;

	if (millisec == osaWaitForever_c) {
		ret = k_sem_take(sem, K_FOREVER);
	} else {
		ret = k_sem_take(sem, K_MSEC(millisec));
	}
	if (ret == -EAGAIN || ret == -EBUSY) {
		return KOSA_StatusTimeout;
	}
	return KOSA_StatusSuccess;
}

/*!
 * @brief Signals for someone waiting on the semaphore to wake up.
 *
 * Wakes up one task that is waiting on the semaphore. If no task is waiting, increases
 * the semaphore's counting value.
 *
 * @param semaphoreHandle The semaphore handle to signal.
 *
 * @retval KOSA_StatusSuccess The semaphore is successfully signaled.
 * @retval KOSA_StatusError   The object can not be signaled or invalid parameter.
 *
 */
osa_status_t OSA_SemaphorePost(osa_semaphore_handle_t semaphoreHandle)
{
	struct k_sem *sem = (struct k_sem *)semaphoreHandle;

	k_sem_give(sem);
	return KOSA_StatusSuccess;
}

/*!
 * @brief Create an unlocked mutex.
 *
 * This function creates a non-recursive mutex and sets it to unlocked status.
 *
 * Example below shows how to use this API to create the mutex handle.
 * @code
 *   OSA_MUTEX_HANDLE_DEFINE(mutexHandle);
 *   OSA_MutexCreate((osa_mutex_handle_t)mutexHandle);
 * @endcode
 *
 * @param mutexHandle       Pointer to a memory space of size
 * OSA_MUTEX_HANDLE_SIZE allocated by the caller. The handle should be 4 byte aligned,
 * because unaligned access doesn't be supported on some devices.
 * You can define the handle in the following two ways:
 * #OSA_MUTEX_HANDLE_DEFINE(mutexHandle);
 * or
 * uint32_t mutexHandle[((OSA_MUTEX_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
 * @retval KOSA_StatusSuccess  the new mutex if the mutex is created successfully.
 * @retval KOSA_StatusError   if the mutex can not be created.
 */
osa_status_t OSA_MutexCreate(osa_mutex_handle_t mutexHandle)
{
	struct k_mutex *mutex = (struct k_mutex *)mutexHandle;

	if (k_mutex_init(mutex)) {
		return KOSA_StatusError;
	}
	return KOSA_StatusSuccess;
}

/*!
 * @brief Waits for a mutex and locks it.
 *
 * This function checks the mutex's status. If it is unlocked, locks it and returns the
 * KOSA_StatusSuccess. Otherwise, waits for a timeout in milliseconds to lock.
 *
 * @param mutexHandle The mutex handle.
 * @param millisec The maximum number of milliseconds to wait for the mutex.
 *                 If the mutex is locked, Pass the value osaWaitForever_c will
 *                 wait indefinitely, pass 0 will return KOSA_StatusTimeout
 *                 immediately.
 *
 * @retval KOSA_StatusSuccess The mutex is locked successfully.
 * @retval KOSA_StatusTimeout Timeout occurred.
 * @retval KOSA_StatusError   Incorrect parameter was passed.
 *
 * @note This is non-recursive mutex, a task can not try to lock the mutex it has locked.
 */
osa_status_t OSA_MutexLock(osa_mutex_handle_t mutexHandle, uint32_t millisec)
{
	struct k_mutex *mutex = (struct k_mutex *)mutexHandle;
	int ret;

	if (millisec == osaWaitForever_c) {
		ret = k_mutex_lock(mutex, K_FOREVER);
	} else {
		ret = k_mutex_lock(mutex, K_MSEC(millisec));
	}
	if (ret) {
		return KOSA_StatusTimeout;
	}
	return KOSA_StatusSuccess;
}

/*!
 * @brief Unlocks a previously locked mutex.
 *
 * @param mutexHandle The mutex handle.
 *
 * @retval KOSA_StatusSuccess The mutex is successfully unlocked.
 * @retval KOSA_StatusError   The mutex can not be unlocked or invalid parameter.
 */
osa_status_t OSA_MutexUnlock(osa_mutex_handle_t mutexHandle)
{
	struct k_mutex *mutex = (struct k_mutex *)mutexHandle;

	if (k_mutex_unlock(mutex)) {
		return KOSA_StatusError;
	}
	return KOSA_StatusSuccess;
}


/*!
 * @brief Destroys a previously created mutex.
 *
 * @param mutexHandle The mutex handle.
 *
 * @retval KOSA_StatusSuccess The mutex is successfully destroyed.
 * @retval KOSA_StatusError   The mutex can not be destroyed.
 *
 */
osa_status_t OSA_MutexDestroy(osa_mutex_handle_t mutexHandle)
{
	return KOSA_StatusSuccess;
}

/*!
 * @brief Initializes an event object with all flags cleared.
 *
 * This function creates an event object and set its clear mode. If autoClear
 * is 1, when a task gets the event flags, these flags will be
 * cleared automatically. Otherwise these flags must
 * be cleared manually.
 *
 * Example below shows how to use this API to create the event handle.
 * @code
 *   OSA_EVENT_HANDLE_DEFINE(eventHandle);
 *   OSA_EventCreate((osa_event_handle_t)eventHandle, 0);
 * @endcode
 *
 * @param eventHandle Pointer to a memory space of size OSA_EVENT_HANDLE_SIZE
 * allocated by the caller. The handle should be 4 byte aligned, because unaligned access
 * doesn't be supported on some devices.
 * You can define the handle in the following two ways:
 * #OSA_EVENT_HANDLE_DEFINE(eventHandle);
 * or
 * uint32_t eventHandle[((OSA_EVENT_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
 * @param autoClear 1 The event is auto-clear.
 *                  0 The event manual-clear
 * @retval KOSA_StatusSuccess  the new event if the event is created successfully.
 * @retval KOSA_StatusError   if the event can not be created.
 */
osa_status_t OSA_EventCreate(osa_event_handle_t eventHandle, uint8_t autoClear)
{
	struct zephyr_event *event = (struct zephyr_event *)eventHandle;

	k_poll_signal_init(&event->signal);
	k_poll_event_init(&event->event, K_POLL_TYPE_SIGNAL,
			  K_POLL_MODE_NOTIFY_ONLY, &event->signal);
	return KOSA_StatusSuccess;
}



/*!
 * @brief Sets one or more event flags.
 *
 * Sets specified flags of an event object.
 *
 * @param eventHandle     The event handle.
 * @param flagsToSet  Flags to be set.
 *
 * @retval KOSA_StatusSuccess The flags were successfully set.
 * @retval KOSA_StatusError   An incorrect parameter was passed.
 */
osa_status_t OSA_EventSet(osa_event_handle_t eventHandle, osa_event_flags_t flagsToSet)
{
	struct zephyr_event *event = (struct zephyr_event *)eventHandle;

	/* Set the requested flags in the flag_state variable */
	event->flag_state |= flagsToSet;
	if (k_poll_signal_raise(&event->signal, event->flag_state)) {
		return KOSA_StatusError;
	}
	return KOSA_StatusSuccess;
}

/*!
 * @brief Clears one or more flags.
 *
 * Clears specified flags of an event object.
 *
 * @param eventHandle       The event handle.
 * @param flagsToClear  Flags to be clear.
 *
 * @retval KOSA_StatusSuccess The flags were successfully cleared.
 * @retval KOSA_StatusError   An incorrect parameter was passed.
 */
osa_status_t OSA_EventClear(osa_event_handle_t eventHandle, osa_event_flags_t flagsToClear)
{
	struct zephyr_event *event = (struct zephyr_event *)eventHandle;

	/* Clear the flag in the event. When the event signal is raised again,
	 * this cleared value will be passed on to the signal result value
	 */
	event->flag_state &= ~flagsToClear;
	return KOSA_StatusSuccess;
}

/*!
 * @brief Get event's flags.
 *
 * Get specified flags of an event object.
 *
 * @param eventHandle       The event handle.
 * The macro EVENT_HANDLE_BUFFER_GET is used to get the event buffer pointer,
 * and should not be used before the macro EVENT_HANDLE_BUFFER_DEFINE is used.
 * @param flagsMask         The flags user want to get are specified by this parameter.
 * @param pFlagsOfEvent     The event flags are obtained by this parameter.
 *
 * @retval KOSA_StatusSuccess The event flags were successfully got.
 * @retval KOSA_StatusError   An incorrect parameter was passed.
 */
osa_status_t OSA_EventGet(osa_event_handle_t eventHandle,
			  osa_event_flags_t flagsMask,
			  osa_event_flags_t *pFlagsOfEvent)
{
	struct zephyr_event *event = (struct zephyr_event *) eventHandle;

	*pFlagsOfEvent = (flagsMask & event->flag_state);
	return KOSA_StatusSuccess;
}


/*!
 * @brief Waits for specified event flags to be set.
 *
 * This function waits for a combination of flags to be set in an event object.
 * Applications can wait for any/all bits to be set. Also this function could
 * obtain the flags who wakeup the waiting task.
 *
 * @param eventHandle     The event handle.
 * @param flagsToWait Flags that to wait.
 * @param waitAll     Wait all flags or any flag to be set.
 * @param millisec    The maximum number of milliseconds to wait for the event.
 *                    If the wait condition is not met, pass osaWaitForever_c will
 *                    wait indefinitely, pass 0 will return KOSA_StatusTimeout
 *                    immediately.
 * @param pSetFlags    Flags that wakeup the waiting task are obtained by this parameter.
 *
 * @retval KOSA_StatusSuccess The wait condition met and function returns successfully.
 * @retval KOSA_StatusTimeout Has not met wait condition within timeout.
 * @retval KOSA_StatusError   An incorrect parameter was passed.

 *
 * @note    Please pay attention to the flags bit width, FreeRTOS uses the most
 *          significant 8 bis as control bits, so do not wait these bits while using
 *          FreeRTOS.
 *
 */
osa_status_t OSA_EventWait(osa_event_handle_t eventHandle,
			   osa_event_flags_t flagsToWait,
			   uint8_t waitAll,
			   uint32_t millisec,
			   osa_event_flags_t *pSetFlags)
{
	struct zephyr_event *event = (struct zephyr_event *) eventHandle;
	int64_t uptime;
	int ret;
	osa_event_flags_t flags = 0U;

	do {
		if (millisec == osaWaitForever_c) {
			ret = k_poll(&event->event, 1, K_FOREVER);
		} else {
			uptime = k_uptime_get();
			ret = k_poll(&event->event, 1, K_MSEC(millisec));
		}
		if (ret == -EAGAIN) {
			return KOSA_StatusTimeout;
		} else if (ret == -EINTR) {
			return KOSA_StatusError;
		}
		/* Look at event bits value to see what has been set */
		if (waitAll) {
			flags |= event->flag_state;
			if ((flags & flagsToWait) == flagsToWait) {
				*pSetFlags = flagsToWait;
				return KOSA_StatusSuccess;
			}
		} else {
			flags = event->flag_state;
			if (flags & flagsToWait) {
				*pSetFlags = flags;
				return KOSA_StatusSuccess;
			}
		}
		/* If wait is not infinite, decrement the number of ms to
		 * wait before timeout.
		 */
		if (millisec != osaWaitForever_c) {
			millisec -= (k_uptime_get() - uptime);
		}
	} while (millisec != 0);
	return KOSA_StatusTimeout;
}

/*!
 * @brief Destroys a previously created event object.
 *
 * @param eventHandle The event handle.
 *
 * @retval KOSA_StatusSuccess The event is successfully destroyed.
 * @retval KOSA_StatusError   Event destruction failed.
 */
osa_status_t OSA_EventDestroy(osa_event_handle_t eventHandle)
{
	struct zephyr_event *event = (struct zephyr_event *)eventHandle;

	k_poll_signal_reset(&event->signal);
	return KOSA_StatusSuccess;
}

/*!
 * @brief Initializes a message queue.
 *
 * This function  allocates memory for and initializes a message queue.
 * Message queue elements are hardcoded as void*.
 *
 * Example below shows how to use this API to create the massage queue handle.
 * @code
 *   OSA_MSGQ_HANDLE_DEFINE(msgqHandle);
 *   OSA_MsgQCreate((osa_msgq_handle_t)msgqHandle, 5U, sizeof(msg));
 * @endcode
 *
 * @param msgqHandle    Pointer to a memory space of size #(OSA_MSGQ_HANDLE_SIZE + msgNo*msgSize)
 * on bare-matel and #(OSA_MSGQ_HANDLE_SIZE) on FreeRTOS allocated by the caller,
 * message queue handle. The handle should be 4 byte aligned, because unaligned
 * access doesn't be supported on some devices.
 * You can define the handle in the following two ways:
 * #OSA_MSGQ_HANDLE_DEFINE(msgqHandle);
 * or
 * For bm: uint32_t
 * msgqHandle[((OSA_MSGQ_HANDLE_SIZE + msgNo*msgSize + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
 * For freertos: uint32_t
 * msgqHandle[((OSA_MSGQ_HANDLE_SIZE + sizeof(uint32_t) - 1U) / sizeof(uint32_t))];
 * @param msgNo :number of messages the message queue should accommodate.
 * @param msgSize :size of a single message structure.
 *
 * @retval KOSA_StatusSuccess Message queue successfully Create.
 * @retval KOSA_StatusError     Message queue create failure.
 */
osa_status_t OSA_MsgQCreate(osa_msgq_handle_t msgqHandle, uint32_t msgNo, uint32_t msgSize)
{
	struct zephyr_queue *queue = (struct zephyr_queue *)msgqHandle;

	(void)msgNo;
	(void)msgSize;
	k_queue_init(&queue->queue);
	queue->msg_size = msgSize;
	queue->msg_cnt = 0;
	return KOSA_StatusSuccess;
}

/*!
 * @brief Puts a message at the end of the queue.
 *
 * This function puts a message to the end of the message queue. If the queue
 * is full, this function returns the KOSA_StatusError;
 *
 * @param msgqHandle  Message Queue handler.
 * @param pMessage Pointer to the message to be put into the queue.
 *
 * @retval KOSA_StatusSuccess Message successfully put into the queue.
 * @retval KOSA_StatusError   The queue was full or an invalid parameter was passed.
 */
osa_status_t OSA_MsgQPut(osa_msgq_handle_t msgqHandle, osa_msg_handle_t pMessage)
{
	struct zephyr_queue *queue = (struct zephyr_queue *)msgqHandle;

	if (k_queue_alloc_append(&queue->queue, pMessage)) {
		return KOSA_StatusError;
	}
	queue->msg_cnt++;
	return KOSA_StatusSuccess;
}

/*!
 * @brief Reads and remove a message at the head of the queue.
 *
 * This function gets a message from the head of the message queue. If the
 * queue is empty, timeout is used to wait.
 *
 * @param msgqHandle   Message Queue handler.
 * @param pMessage Pointer to a memory to save the message.
 * @param millisec The number of milliseconds to wait for a message. If the
 *                 queue is empty, pass osaWaitForever_c will wait indefinitely,
 *                 pass 0 will return KOSA_StatusTimeout immediately.
 *
 * @retval KOSA_StatusSuccess   Message successfully obtained from the queue.
 * @retval KOSA_StatusTimeout   The queue remains empty after timeout.
 * @retval KOSA_StatusError     Invalid parameter.
 */
osa_status_t OSA_MsgQGet(osa_msgq_handle_t msgqHandle, osa_msg_handle_t pMessage,
			uint32_t millisec)
{
	struct zephyr_queue *queue = (struct zephyr_queue *)msgqHandle;
	void *msg;

	if (millisec == osaWaitForever_c) {
		msg = k_queue_get(&queue->queue, K_FOREVER);
	} else {
		msg = k_queue_get(&queue->queue, K_MSEC(millisec));
	}
	if (msg == NULL) {
		return KOSA_StatusTimeout;
	}
	memcpy(pMessage, msg, queue->msg_size);
	queue->msg_cnt--;
	return KOSA_StatusSuccess;
}




/*!
 * @brief Get the available message
 *
 * This function is used to get the available message.
 *
 * @param msgqHandle Message Queue handler.
 *
 * @return Available message count
 */
int OSA_MsgQAvailableMsgs(osa_msgq_handle_t msgqHandle)
{
	struct zephyr_queue *queue = (struct zephyr_queue *)msgqHandle;

	return queue->msg_cnt;
}



/*!
 * @brief Destroys a previously created queue.
 *
 * @param msgqHandle Message Queue handler.
 *
 * @retval KOSA_StatusSuccess The queue was successfully destroyed.
 * @retval KOSA_StatusError   Message queue destruction failed.
 */
osa_status_t OSA_MsgQDestroy(osa_msgq_handle_t msgqHandle)
{
	struct zephyr_queue *queue = (struct zephyr_queue *)msgqHandle;

	/* Drain the queue */
	while (k_queue_get(&queue->queue, K_NO_WAIT) != NULL) {
		;
	}
	queue->msg_size = 0;
	queue->msg_cnt = 0;
	return KOSA_StatusSuccess;
}

/*!
 * @brief Enable all interrupts.
 */
void OSA_InterruptEnable(void)
{
	irq_unlock(gIrqKey);
}

/*!
 * @brief Disable all interrupts.
 */
void OSA_InterruptDisable(void)
{
	gIrqKey = irq_lock();
}

/*!
 * @brief Enable all interrupts using PRIMASK.
 */
void OSA_EnableIRQGlobal(void)
{
	irq_unlock(gIrqKey);
}

/*!
 * @brief Disable all interrupts using PRIMASK.
 */
void OSA_DisableIRQGlobal(void)
{
	gIrqKey = irq_lock();
}

/*!
 * @brief Delays execution for a number of milliseconds.
 *
 * @param millisec The time in milliseconds to wait.
 */
void OSA_TimeDelay(uint32_t millisec)
{
	k_msleep(millisec);
}

/*!
 * @brief This function gets current time in milliseconds.
 *
 * @retval current time in milliseconds
 */
uint32_t OSA_TimeGetMsec(void)
{
	return k_uptime_get_32();
}

