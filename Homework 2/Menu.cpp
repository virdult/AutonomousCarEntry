#include "Menu.hpp"
#include <fstream>
#include <limits>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

using nlohmann::json;

namespace menu {

// ---------- MenuItem ----------
MenuItem::MenuItem(std::string name, double price, std::array<double, 5> taste)
    : name(std::move(name)), price(price), taste(taste) {}

std::string MenuItem::getName() const { return name; }
double MenuItem::getPrice() const { return price; }
std::array<double, 5> MenuItem::getTaste() const { return taste; }

void MenuItem::setName(std::string n) { name = std::move(n); }
void MenuItem::setPrice(double p) { price = p; }
void MenuItem::setTaste(const std::array<double, 5>& t) { taste = t; }

// ---------- Derived Classes ----------
Starter::Starter(std::string name, double price, std::array<double, 5> taste, bool isHot)
    : MenuItem(std::move(name), price, taste), isHot(isHot) {}

void Starter::printInfo() const {
    std::cout << std::left << std::setw(20) << "Starter: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice()
              << " (" << (isHot ? "Hot" : "Cold") << ")\n";
}

json Starter::toJson() const {
    return {{"type", "Starter"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"isHot", isHot}};
}

Salad::Salad(std::string name, double price, std::array<double, 5> taste, bool hasTopping)
    : MenuItem(std::move(name), price, taste), hasTopping(hasTopping) {}

void Salad::printInfo() const {
    std::cout << std::left << std::setw(20) << "Salad: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice();
    if (hasTopping) std::cout << " (+Topping)";
    std::cout << "\n";
}

json Salad::toJson() const {
    return {{"type", "Salad"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"hasTopping", hasTopping}};
}

MainCourse::MainCourse(std::string name, double price, std::array<double, 5> taste, bool isVegetarian)
    : MenuItem(std::move(name), price, taste), isVegetarian(isVegetarian) {}

void MainCourse::printInfo() const {
    std::cout << std::left << std::setw(20) << "Main Course: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice()
              << " (" << (isVegetarian ? "Vegetarian" : "Non-Vegetarian") << ")\n";
}

json MainCourse::toJson() const {
    return {{"type", "MainCourse"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"isVegetarian", isVegetarian}};
}

Drink::Drink(std::string name, double price, std::array<double, 5> taste,
             bool isCarbonated, bool hasAlcohol)
    : MenuItem(std::move(name), price, taste), isCarbonated(isCarbonated), hasAlcohol(hasAlcohol) {}

void Drink::printInfo() const {
    std::cout << std::left << std::setw(20) << "Drink: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice() << " ("
              << (isCarbonated ? "Carbonated" : "Still");
    if (hasAlcohol) std::cout << ", Alcoholic";
    std::cout << ")\n";
}

json Drink::toJson() const {
    return {{"type", "Drink"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"isCarbonated", isCarbonated}, {"hasAlcohol", hasAlcohol}};
}

Appetizer::Appetizer(std::string name, double price, std::array<double, 5> taste, std::string serveTime)
    : MenuItem(std::move(name), price, taste), serveTime(std::move(serveTime)) {}

void Appetizer::printInfo() const {
    std::cout << std::left << std::setw(20) << "Appetizer: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice()
              << " (Serve " << serveTime << " main course)\n";
}

json Appetizer::toJson() const {
    return {{"type", "Appetizer"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"serveTime", serveTime}};
}

Dessert::Dessert(std::string name, double price, std::array<double, 5> taste, bool extraChocolate)
    : MenuItem(std::move(name), price, taste), extraChocolate(extraChocolate) {}

void Dessert::printInfo() const {
    std::cout << std::left << std::setw(20) << "Dessert: " << std::setw(40) << getName() << " - $" << std::setw(5) << getPrice();
    if (extraChocolate) std::cout << " (+Extra Chocolate)";
    std::cout << "\n";
}

json Dessert::toJson() const {
    return {{"type", "Dessert"}, {"name", getName()}, {"price", getPrice()},
            {"taste", getTaste()}, {"extraChocolate", extraChocolate}};
}

// ---------- Menu ----------
void Menu::addItem(MenuItem* item) {
    items.push_back(item);
}

void Menu::removeItem(int index) {
    if (index < 0 || index >= static_cast<int>(items.size())) {
        std::cout << "Invalid index!\n";
        return;
    }
    delete items[index];
    items.erase(items.begin() + index);
    std::cout << "Item removed.\n";
}

void Menu::showMenu() const {
    if (items.empty()) {
        std::cout << "\nYour menu is empty.\n";
        return;
    }
    std::cout << "\n----- Your Menu -----\n";
    for (size_t i = 0; i < items.size(); ++i) {
        std::cout << std::left << std::setw(4) << (std::to_string(i + 1) + ". ");
        items[i]->printInfo();
    }
    std::cout << "--------------------" << "\n";
    std::cout << "Total Cost: $" << totalCost() << "\n";
}

std::vector<menu::MenuItem*> Menu::showMenuByType(const std::vector<menu::MenuItem*>& items, const std::string& type) {
    std::cout << "\n--- " << type << " ---\n";
    std::vector<menu::MenuItem*> filtered;
    for (auto* item : items) {
        bool match = false;
        if (type == "Starter" && dynamic_cast<menu::Starter*>(item)) match = true;
        else if (type == "Salad" && dynamic_cast<menu::Salad*>(item)) match = true;
        else if (type == "MainCourse" && dynamic_cast<menu::MainCourse*>(item)) match = true;
        else if (type == "Drink" && dynamic_cast<menu::Drink*>(item)) match = true;
        else if (type == "Appetizer" && dynamic_cast<menu::Appetizer*>(item)) match = true;
        else if (type == "Dessert" && dynamic_cast<menu::Dessert*>(item)) match = true;

        if (match) {
            filtered.push_back(item);
        }
    }

    if (filtered.empty()) {
        std::cout << "No items available in this category.\n";
        return filtered;
    }

    for (size_t i = 0; i < filtered.size(); ++i) {
        std::cout << i + 1 << ". ";
        filtered[i]->printInfo();
    }

    return filtered;
}

double Menu::totalCost() const {
    double total = 0.0;
    for (auto* item : items) total += item->getPrice();
    return total;
}

int Menu::getItemCount() const {
    return static_cast<int>(items.size());
}

std::array<double, 5> Menu::getItemTaste(int index) const {
    if (index < 0 || index >= static_cast<int>(items.size()))
        return {0, 0, 0, 0, 0};
    return items[index]->getTaste();
}

void Menu::saveToFile(const std::string& filename) const {
    json j;
    for (auto* item : items)
        j["items"].push_back(item->toJson());
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing.\n";
        return;
    }
    file << j.dump(4);
    std::cout << "Menu saved to " << filename << "\n";
}

void Menu::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "No saved menu found (" << filename << ").\n";
        return;
    }

    json j;
    file >> j;
    file.close();

    for (auto* old : items) delete old;
    items.clear();

    if (!j.contains("items") || !j["items"].is_array()) {
        std::cout << "Invalid menu file format.\n";
        return;
    }

    for (const auto& item : j["items"]) {
        std::string type = item.value("type", "");
        std::array<double, 5> taste = {0, 0, 0, 0, 0};

        if (item.contains("taste") && item["taste"].is_array()) {
            for (size_t i = 0; i < taste.size() && i < item["taste"].size(); ++i)
                taste[i] = item["taste"][i].get<double>();
        }

        if (type == "Starter")
            addItem(new Starter(item.value("name", ""), item.value("price", 0.0), taste, item.value("isHot", false)));
        else if (type == "Salad")
            addItem(new Salad(item.value("name", ""), item.value("price", 0.0), taste, item.value("hasTopping", false)));
        else if (type == "MainCourse")
            addItem(new MainCourse(item.value("name", ""), item.value("price", 0.0), taste, item.value("isVegetarian", false)));
        else if (type == "Drink")
            addItem(new Drink(item.value("name", ""), item.value("price", 0.0), taste,
                              item.value("isCarbonated", false), item.value("hasAlcohol", false)));
        else if (type == "Appetizer")
            addItem(new Appetizer(item.value("name", ""), item.value("price", 0.0), taste, item.value("serveTime", "before")));
        else if (type == "Dessert")
            addItem(new Dessert(item.value("name", ""), item.value("price", 0.0), taste, item.value("extraChocolate", false)));
    }

    std::cout << "Menu loaded from " << filename << "\n";
}

Menu::~Menu() {
    for (auto* item : items) delete item;
}

// ---------- User ----------
static std::string capitalize(const std::string& input) {
    if (input.empty()) return input;
    std::string result = input;
    result[0] = std::toupper(result[0]);
    for (size_t i = 1; i < result.size(); ++i)
        result[i] = std::tolower(result[i]);
    return result;
}

void User::setUserInfo() {
    // Use safe getline pattern
    std::string tmp;
    std::cout << "Enter first name: ";
    std::getline(std::cin, firstName);
    firstName = capitalize(firstName);

    std::cout << "Enter last name: ";
    std::getline(std::cin, lastName);
    lastName = capitalize(lastName);

    while (true) {
        std::cout << "Enter gender (Mr or Mrs): ";
        std::getline(std::cin, gender);

        std::string genderLower = gender;
        std::transform(genderLower.begin(), genderLower.end(), genderLower.begin(), [](unsigned char c){ return std::tolower(c); });

        if (genderLower == "mr" || genderLower == "mrs") {
            gender[0] = std::toupper(gender[0]);
            break;
        }

        std::cout << "Invalid input. Please enter 'Mr' or 'Mrs' only.\n";
    }
}

void User::showUserInfo() const {
    std::cout << "\nHello " << gender << " " << lastName << "!\n";
}

Menu& User::getMenu() {
    return userMenu;
}

json User::toJson() const {
    return {{"firstName", firstName}, {"lastName", lastName}, {"gender", gender}};
}

void User::fromJson(const json& j) {
    firstName = j.value("firstName", "");
    lastName = j.value("lastName", "");
    gender = j.value("gender", "");
}

// --- Minimal accessors for multi-user handling ---
void User::setFirstName(const std::string &n) {
    firstName = n;
}
void User::setLastName(const std::string &n) {
    lastName = n;
}
void User::setGender(const std::string &g) {
    gender = g;
}
std::string User::getFirstName() const {
    return firstName;
}
std::string User::getLastName() const {
    return lastName;
}

} // namespace menu
