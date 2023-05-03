#pragma once

// Adapted from: https://www.geeksforgeeks.org/implement-thread-safe-queue-in-c/

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <chrono>

// Thread-safe queue
template <typename T>
class TSQueue
{
	private:
		// Underlying queue
		std::queue<T> m_queue;

		// mutex for thread synchronization
		std::mutex m_mutex;

		// Condition variable for signaling
		std::condition_variable m_cond;

		// Queue size limit
		size_t max_size;

		// Overwrite on push when full
		bool overwrite_when_full;

	public:
		// Construct queue with optional size limit
		TSQueue(size_t max_size = 0, bool overwrite_when_full = false): max_size(max_size), overwrite_when_full(overwrite_when_full) {};

		// Pushes an element to the queue
		bool push(T item, bool blocking = true, unsigned int timeout_us = 0)
		{
			// Acquire lock
			std::unique_lock<std::mutex> lock(m_mutex);

			// check if queue full
			if ((0 != max_size) && (m_queue.size() == max_size))
			{
				// it is, consider waiting until queue is not full
				if (blocking) {
					// blocking allowed, is timeout supplied?
					if (0 == timeout_us) {
						// Infinite wait
						m_cond.wait(lock,
									[this]() { return (m_queue.size() < max_size); });
					} else {
						// Timed wait
						if (!m_cond.wait_for(lock,
											std::chrono::microseconds(timeout_us),
											[this]() { return (m_queue.size() < max_size); })
						   )
						{
							// Queue still full after timeout, can we overwrite an entry?
							if (!overwrite_when_full) {
								// Overwriting not allowed, item not pushed
								return false;
							}

							// Drop oldest entry
							m_queue.pop();
						}
					}
				} else {
					// No blocking allowed, can we overwrite?
					if (!overwrite_when_full) {
						// Overwriting not allowed, item not pushed
						return false;
					}

					// Drop oldest entry
					m_queue.pop();
				}
			}

			// Add item
			m_queue.push(item);

			// Notify one thread that is waiting
			m_cond.notify_one();

			// Item pushed
			return true;
		}

		// Pops an element off the queue
		bool pop(T &item, bool blocking = true, unsigned int timeout_us = 0)
		{
			// acquire lock
			std::unique_lock<std::mutex> lock(m_mutex);

			// wait until queue is not empty
			if (m_queue.empty()) {
				// Queue empty, can we block?
				if (blocking) {
					if (0 == timeout_us) {
						// Infinite wait
						m_cond.wait(lock,
									[this]() { return !m_queue.empty(); });
					} else {
						// Timed wait
						if (!m_cond.wait_for(lock,
											std::chrono::microseconds(timeout_us),
											[this]() { return !m_queue.empty(); })
						   )
						{
							// Timeout
							return false;
						}
					}
				} else {
					// Not allowed to block and queue empty, no item available
					return false;
				}
			}

			// retrieve item
			item = m_queue.front();
			m_queue.pop();

			// Notify one thread that is waiting
			m_cond.notify_one();

			// item popped
			return true;
		}
};
