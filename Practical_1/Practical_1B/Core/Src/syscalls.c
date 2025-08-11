#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>

extern char _end; // Provided by linker: end of .bss/.data; start of heap
static char *heap_end = 0;

int _close(int file)
{
    (void)file;
    errno = ENOSYS;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    if (st == 0) {
        errno = EFAULT;
        return -1;
    }
    st->st_mode = S_IFCHR; // Character device (e.g., stdout/stderr)
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1; // Treat all fds as TTY
}

int _lseek(int file, int ptr, int dir)
{
    (void)file; (void)ptr; (void)dir;
    errno = ENOSYS;
    return -1;
}

int _read(int file, char *ptr, int len)
{
    (void)file; (void)ptr; (void)len;
    // No input device; report EOF
    return 0;
}

int _write(int file, const char *ptr, int len)
{
    (void)file; (void)ptr;
    // Discard output but report success to avoid blocking stdio users
    return len;
}

void* _sbrk(int incr)
{
    if (heap_end == 0) {
        heap_end = &_end;
    }

    char *prev_heap_end = heap_end;

    // Get current stack pointer
    register char *stack_ptr __asm__("sp");

    if (heap_end + incr > stack_ptr) {
        // Out of memory
        errno = ENOMEM;
        return (void*)-1;
    }

    heap_end += incr;
    return prev_heap_end;
} 