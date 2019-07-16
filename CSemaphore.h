#ifndef OS_CSEMAPHORE_H
#define OS_CSEMAPHORE_H


#include <semaphore.h>
#include <string>


namespace DAQ {
namespace OS {

class CSemaphore;


/*! \brief CScopedWait class
 *
 *  The CScopedWait class is a very simple class that used RAII to
 *  make sure that a post() gets called after a wait().
 *
 * \code
 *  DAQ::OS::CPosixSemaphore sem("asdf", N);
 *
 *  {
 *      // semaphore count is N
 *      DAQ::OS::CScopedWait guard(sem);
 *      // semaphore count is  N-1
 *  }
 *  // semaphore count is N
 *
 * \endcode
 *
 */
class CScopedWait
{
  private:
    CSemaphore& m_sem;

  public:
    CScopedWait(CSemaphore& sem);
    CScopedWait(const CScopedWait& rhs) = delete;
    ~CScopedWait();
};



/*!
 * \brief The CSemaphore class
 *
 * An abstract base class for all semaphores.
 *
 */
class CSemaphore
{

public:
    virtual void wait() = 0;
    virtual void timedWait(int nMilliseconds) = 0;

    virtual bool tryWait() = 0;

    virtual void post() = 0;

    virtual int getCount() const = 0;
};



/*!
 * \brief The CPosixSemaphore class
 *
 * A concrete implementation of the CSemaphore class. This
 * class assumes standard, POSIX semaphores. This implementation is
 * based on the libc implementation. An example usage of this is:
 *
 * \code
 *  CPosixSemaphore sem("asdf",1);
 *  sem.wait();
 *
 *  // do whatever required synchronization
 *
 *  sem.post();
 * \endcode
 *
 * The semantics of these semaphores is that CPosixSemaphore::wait() decrements
 * the count and CPosixSemaphore::post() increments it. If the count of the
 * semaphore is 0, calls to CPosixSemaphore::wait() will cause the process to block.
 * Once the semaphore count becomes nonzero, one process will be awoke. At any
 * point in time, the semaphore count can be acquired with
 * CPosixSemaphore::getCount().
 *
 * The goal of the implementation is also to not leave any dangling semaphores around
 * when no process requires it. To accomplish this the semaphore is closed and unlinked
 * in the destructor.
 *
 */
class CPosixSemaphore : public CSemaphore
{
    // see source file for further annotation of methods

private:
    sem_t*      m_pSem;     ///! handle to semaphore
    std::string m_name;     ///! name of semaphore

private:
    CPosixSemaphore(const CPosixSemaphore& rhs) = delete;
    CPosixSemaphore& operator=(const CPosixSemaphore&);

public:
    CPosixSemaphore(const std::string& name, int count);

    virtual ~CPosixSemaphore();

    virtual void wait();
    virtual void timedWait(int nMilliseconds);

    virtual bool tryWait();
    virtual void post();

    virtual int getCount() const;

    sem_t* getNativeHandle() { return m_pSem; }
};


} // end OS namespace
} // end DAQ namespace


#endif // CSYSTEMSEMAPHORE_H
