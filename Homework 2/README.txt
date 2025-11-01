# Restaurant Recommender Bot

A C++ console application that recommends restaurant menu items based on a user's taste preferences.  
It uses a simple linear regression model to predict satisfaction from flavor profiles such as sweet, sour, bitter, salty, and savory.

---

## Overview

This project simulates a restaurant assistant that:
- Loads menu data from `menu.json`.
- Lets users register with taste preferences.
- Uses a small AI model to suggest the best dish.
- Saves user data to `user_data.json`.

---

## Features

- Interactive text-based menu
- User profile creation and saving
- Basic AI recommender (linear regression)
- JSON data integration (via nlohmann/json)

---

## Project Structure

RestaurantBot/
├── CMakeLists.txt
├── main.cpp
├── Menu.cpp
├── Menu.hpp
├── Recommender.hpp
├── menu.json
├── user_data.json
├── weights.json
└── build/
---

## Dependencies

- C++17 or later
- CMake 3.10+
- nlohmann/json

Install JSON on Ubuntu/Debian:
```bash
sudo apt install nlohmann-json3-dev

---

Build Instructions

mkdir build && cd build
cmake ..
make
./RestaurantBot

Author: Virdult
