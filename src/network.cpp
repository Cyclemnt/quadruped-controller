#include "../include/network.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <algorithm>
#include "../include/control_flags.hpp"

// sanitize minimal
static std::string sanitize(const std::string& s) {
    std::string r = s;
    r.erase(std::remove(r.begin(), r.end(), '\r'), r.end());
    r.erase(std::remove(r.begin(), r.end(), '\n'), r.end());
    while (!r.empty() && isspace((unsigned char)r.front())) r.erase(r.begin());
    while (!r.empty() && isspace((unsigned char)r.back())) r.pop_back();
    return r;
}

void network_thread_func(int port) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::perror("socket");
        return;
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::perror("setsockopt");
        close(server_fd);
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        close(server_fd);
        return;
    }

    if (listen(server_fd, 5) < 0) {
        std::perror("listen");
        close(server_fd);
        return;
    }

    std::cout << "[network] Listening on port " << port << std::endl;

    while (true) {
        sockaddr_in client;
        socklen_t client_len = sizeof(client);
        int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client), &client_len);
        if (client_fd < 0) {
            std::perror("accept");
            continue;
        }

        char buf[2048];
        ssize_t n = read(client_fd, buf, sizeof(buf)-1);
        if (n <= 0) {
            close(client_fd);
            continue;
        }
        buf[n] = '\0';
        std::string cmd = sanitize(std::string(buf));
        std::cout << "[network] Received: \"" << cmd << "\"\n";

        // Commands:
        // start_walk_forward / stop_walk_forward
        // start_walk_backward / stop_walk_backward
        // start_walk_right / stop_walk_right
        // start_walk_left / stop_walk_left

        if (cmd == "start_walk_forward") {
            stopRequested.store(false);
            flag_walk_forward.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "stop_walk_forward") {
            flag_walk_forward.store(false);
            // request stop (if routine checks stopRequested)
            stopRequested.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "start_walf_backward") {
            stopRequested.store(false);
            flag_walk_backwards.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "stop_walk_backward") {
            flag_walk_backwards.store(false);
            stopRequested.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "start_walk_right") {
            stopRequested.store(false);
            flag_walk_right.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "stop_walk_right") {
            flag_walk_right.store(false);
            stopRequested.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "start_walk_left") {
            stopRequested.store(false);
            flag_turn_left.store(true);
            write(client_fd, "OK\n", 3);
        } else if (cmd == "start_walk_left") {
            flag_turn_left.store(false);
            stopRequested.store(true);
            write(client_fd, "OK\n", 3);
        } else {
            write(client_fd, "UNKNOWN\n", 8);
        }

        close(client_fd);
    }

    close(server_fd);
    std::cout << "[network] Server closed\n";
}
