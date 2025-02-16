#include <Printf.h>

static void _write(int file, char *ptr, int len)
{

    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        for (i = 0; i < len; i++)
        {
            if (ptr[i] == '/n')
            {
                usart_send_blocking(ptr_usart, '\r');
            }
            usart_send_blocking(ptr_usart, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}
