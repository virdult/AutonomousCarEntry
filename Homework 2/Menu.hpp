#ifndef MENU_HPP
#define MENU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <nlohmann/json.hpp>
#include <cmath>//For ai recommendation

namespace menu {

// Order of taste vector used across the program:
// [0] = sweet
// [1] = sour
// [2] = bitter
// [3] = salty
// [4] = savory
// All taste arrays follow this order for consistency with menu.json.

// Base class representing a menu item.
class MenuItem {
private:
    std::string name;
    double price;
    std::array<double, 5> taste;

public:
    // Constructor
    MenuItem(std::string name = "", double price = 0.0,
             std::array<double, 5> taste = {0, 0, 0, 0, 0});

    // Virtual destructor for safe polymorphic deletion
    virtual ~MenuItem() = default;

    // Getters / setters
    std::string getName() const;
    double getPrice() const;
    std::array<double, 5> getTaste() const;

    void setName(std::string n);
    void setPrice(double p);
    void setTaste(const std::array<double, 5>& t);

    // Print info for this item (polymorphic)
    virtual void printInfo() const = 0;

    // Convert to JSON for persistence (polymorphic)
    virtual nlohmann::json toJson() const = 0;
};

// Derived classes
class Starter : public MenuItem {
private:
    bool isHot;
public:
    Starter(std::string name = "", double price = 0.0,
            std::array<double, 5> taste = {0,0,0,0,0}, bool isHot = true);
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

class Salad : public MenuItem {
private:
    bool hasTopping;
public:
    Salad(std::string name = "", double price = 0.0,
          std::array<double, 5> taste = {0,0,0,0,0}, bool hasTopping = false);
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

class MainCourse : public MenuItem {
private:
    bool isVegetarian;
public:
    MainCourse(std::string name = "", double price = 0.0,
               std::array<double, 5> taste = {0,0,0,0,0}, bool isVegetarian = false);
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

class Drink : public MenuItem {
private:
    bool isCarbonated;
    bool hasAlcohol;
public:
    Drink(std::string name = "", double price = 0.0,
          std::array<double, 5> taste = {0,0,0,0,0}, bool isCarbonated = false, bool hasAlcohol = false);
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

class Appetizer : public MenuItem {
private:
    std::string serveTime; // "before" or "after"
public:
    Appetizer(std::string name = "", double price = 0.0,
              std::array<double, 5> taste = {0,0,0,0,0}, std::string serveTime = "before");
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

class Dessert : public MenuItem {
private:
    bool extraChocolate;
public:
    Dessert(std::string name = "", double price = 0.0,
            std::array<double, 5> taste = {0,0,0,0,0}, bool extraChocolate = false);
    void printInfo() const override;
    nlohmann::json toJson() const override;
};

// Menu container holding MenuItem pointers (ownership)
// This class handles deletion of contained raw pointers.
class Menu {
private:
    std::vector<MenuItem*> items;

public:
    Menu() = default;

    // Add a new item (takes ownership of pointer)
    void addItem(MenuItem* item);

    // Remove item by index
    void removeItem(int index);

    // Print menu contents
    void showMenu() const;

    // Print menu content for choosen type
    std::vector<MenuItem*> showMenuByType(const std::vector<MenuItem*>& items, const std::string& type);

    // Return total cost (sum of item prices)
    double totalCost() const;

    // Number of items
    int getItemCount() const;

    // Return taste vector for item index, or zero vector if invalid
    std::array<double,5> getItemTaste(int index) const;

    // Destructor cleans up owned MenuItem pointers
    ~Menu();

    // Save / load menu contents to JSON file
    void saveToFile(const std::string& filename) const;
    void loadFromFile(const std::string& filename);
};

// User info and their menu
class User {
private:
    std::string firstName;
    std::string lastName;
    std::string gender; // "Mr" or "Mrs"
    Menu userMenu;

public:
    // Ask user info and normalize
    void setUserInfo();

    // Greet user
    void showUserInfo() const;

    // Get reference to stored Menu
    Menu& getMenu();

    // JSON helpers to persist user metadata only (not menu items)
    nlohmann::json toJson() const;
    void fromJson(const nlohmann::json& j);

    // Accessors used by main.cpp for multi-user handling
    void setFirstName(const std::string &n);
    void setLastName(const std::string &n);
    void setGender(const std::string &g);
    std::string getFirstName() const;
    std::string getLastName() const;
};

} // namespace menu

namespace ai {

inline double tasteDistanceWeighted(const std::array<double,5>& a,
                                 const std::array<double,5>& b,
                                 const std::array<double,5>& weights) {
    double sum = 0;
    for (int i = 0; i < 5; ++i) {
        double diff = a[i] - b[i];
        sum += weights[i] * diff * diff;
    }
    return std::sqrt(sum);
}
}

#endif // MENU_HPP