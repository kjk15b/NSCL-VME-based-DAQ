#include "CSemaphore.h"

#include <ErrnoException.h>

#include <fcntl.h>
#include <string.h>
#include <iostream>

namespace DAQ {
namespace OS {


/*!
 * \brief CScopedWait::CScopedWait
 *
 * \param sem   the semaphore to manage
 *
 * The only thing that this does is to call sem.wait()
 */
CScopedWait::CScopedWait(CSemaphore &sem) : m_sem(sem) { m_sem.wait(); }

/*!
 * \brief CScopedWait::~CScopedWait
 *
 * post() is called for the managed semaphore.
 */
CScopedWait::~CScopedWait() { m_sem.post(); }


//////////////////////////////////////////////////////////////////////////////

/// \brief CPosixSemaphore::CPosixSemaphore
///
/// \param name     name of the semaphore
/// \param count    initial count of semaphore if it needs to be created
///
/// If the named semaphore does not exist, it is created and its count is
/// initialized to be count. Otherwise, the existing semaphore is attached
/// to without adjusting its count.
///
/// \throws CErrnoException if sem_open returns a negative status
///
CPosixSemaphore::CPosixSemaphore(const std::string &name, int count)
 : m_pSem(nullptr), m_name(name)
{
    m_pSem = sem_open(m_name.c_str(), O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO, count);
    if (m_pSem == SEM_FAILED) {
        throw CErrnoException(strerror(errno));
    }

}

/*!
 * \brief CPosixSemaphore::~CPosixSemaphore
 *
 * Close and unlink the semaphore.
 */
CPosixSemaphore::~CPosixSemaphore()
{
    sem_close(m_pSem);
    sem_unlink(m_name.c_str());
}

/*!
 * \brief Wait with a timeout
 *
 * \param nMilliseconds the number of milliseconds to wait before timing out
 *
 * If the count of the semaphore is already zero, the thread will block until
 * another holder of the semaphore posts or nMilliseconds passes.
 *
 * \throws CErrnoException if sem_timedwait returns a negative status
 */
void CPosixSemaphore::timedWait(int nMilliseconds)
{

    struct timespec timeout;
    timeout.tv_sec = nMilliseconds/1000;
    timeout.tv_nsec = (nMilliseconds%1000) * 1000000;

    int status = sem_timedwait(m_pSem, &timeout);

    if (status < 0) {
        throw CErrnoException(strerror(errno));
    }
}

/*!
 * \brief Decrement the semaphore count, block if count is zero
 *
 * This will decrement the semaphore count by one. If the count
 * is already zero, the calling thread will block.
 *
 * \throws CErrnoException if sem_wait returns an error
 */
void CPosixSemaphore::wait()
{
    int status = sem_wait(m_pSem);
    if (status < 0) {
        throw CErrnoException(strerror(errno));
    }
}

/*!
 * \brief CPosixSemaphore::tryWait
 *
 * \retval true if semaphore successfully decremented count,
 * \retval false if calling thread would have blocked
 *
 * \throws CErrnoException if sem_trywait returns an error
 */
bool CPosixSemaphore::tryWait()
{
    bool success = false;

    int status = sem_trywait(m_pSem);
    if (status == 0) {
        success = true;
    } else if (status < 0 && errno != EAGAIN) {
        throw CErrnoException(strerror(errno));
    } // else the process returned EAGAIN b/c it would have blocked

    return success;
}

/*!
 * \brief CPosixSemaphore::unlock
 *
 * Attempt to post (i.e. increment) the semaphore.
 *
 * \throws CErrnoException if fails b/c of EINVAL or EOVERFLOW (or anything else)
 *
 */
void CPosixSemaphore::post()
{
    int status = sem_post(m_pSem);
    if (status < 0) {
        throw CErrnoException(strerror(errno));
    }
}

/*!
 * \return the count for the semaphore
 *
 * \throw CErrnoException if sem_getvalue returns an error
 */
int CPosixSemaphore::getCount() const
{
    int count;
    int status = sem_getvalue(m_pSem, &count);

    if (status < 0) {
        throw CErrnoException(strerror(errno));
    }

    return count;
}


} // end OS namespace
} // end DAQ namespace
