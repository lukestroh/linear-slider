#ifndef __CLEARCORE_COMMS_H__
#define __CLEARCORE_COMMS_H__

#include <netinet/in.h>
#include <stdio.h>
#include <string>
#include <sys/socket.h>

#define BUF_LEN_MAX 1024

class ClearCoreComms {
    public:
        bool begin();
        char* read_data();
        void send_data(const char* data);

        ClearCoreComms();
        ~ClearCoreComms();

    private:
        const int local_port = 44644;
        const int remote_port = 8888;
        int sock;
        struct sockaddr_in local_svr_addr;
        struct sockaddr_in client_addr;
        struct sockaddr_in tmp_addr;
        char read_buffer[BUF_LEN_MAX];
        char send_buffer[BUF_LEN_MAX];
        socklen_t client_addr_len = sizeof(client_addr);
        socklen_t tmp_addr_len = sizeof(tmp_addr);

        void construct_send_msg(const char* data);
        
};

#endif // __CLEARCORE_COMMS_H__