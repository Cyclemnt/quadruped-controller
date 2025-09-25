#pragma once
#include <atomic>

// Flags globaux contrôlés par le thread réseau et lus par la boucle principale
extern std::atomic<bool> flag_walk_forward;
extern std::atomic<bool> flag_walk_backwards;
extern std::atomic<bool> flag_walk_right;
extern std::atomic<bool> flag_turn_left;

// Flag d'interruption utilisé *à l'intérieur* des longues routines (walk/run/turn).
// Les fonctions d'action doivent vérifier stopRequested.load() périodiquement.
extern std::atomic<bool> stopRequested;
