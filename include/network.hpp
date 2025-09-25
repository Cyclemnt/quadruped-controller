#pragma once
#include <string>

// Lance le thread réseau (bloquant) ; prévu pour être lancé dans un std::thread.
// Il met à jour les flags déclarés dans include/control_flags.hpp.
void network_thread_func(int port = 12345);
