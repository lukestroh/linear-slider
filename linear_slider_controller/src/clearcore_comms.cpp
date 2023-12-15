#include "linear_slider_controller/clearcore_comms.hpp"
#include "rclcpp/rclcpp.hpp"

ClearCoreComms::ClearCoreComms() {

}

ClearCoreComms::~ClearCoreComms() {

}

bool ClearCoreComms::begin() {
    // Create socket file descriptor
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_FATAL(
            rclcpp::get_logger("LinearSliderCommunicationInterface"),
            "Socket creation failed."
        );
        return false;
    }

    memset(&local_srv_addr, 0, sizeof(local_srv_addr));
    memset(&client_addr, 0, sizeof(client_addr));


    // Set server information
    local_srv_addr.sin_port = htons(remote_port);
    local_srv_addr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket with the server address
    if (bind(sock, (const struct sockaddr*)&local_srv_addr, sizeof(local_srv_addr)) < 0) {
        RCLCPP_FATAL(
            rclcpp::get_logger("LinearSliderCommunicationInterface"),
            "Socket bind failed."
        );
        return false;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("LinearSliderCommunicationInterface"),
        "ClearCoreComms setup complete."
    );

    return true;
}

void ClearCoreComms::read_data() {
    int msg_size;
    msg_size = recvfrom(sock, (char*)read_buffer, BUF_LEN_MAX, MSG_WAITALL, (struct sockaddr*)&client_addr, &client_addr_len);
    read_buffer[msg_size] = '\0';
    return;
}

void ClearCoreComms::construct_send_msg(char* data) {
    /* Fill the send_buffer with data */
    // Empty the send_buffer
    memset(&send_buffer[0], 0, sizeof(send_buffer));
    strcpy(send_buffer, data);
    return;
}

void ClearCoreComms::send_data(char* data) {
    construct_send_msg(data);
    sendto(sock, (const char*)send_buffer, strlen(send_buffer), MSG_CONFIRM, (const struct sockaddr*) &client_addr, client_addr_len);
    return;
}