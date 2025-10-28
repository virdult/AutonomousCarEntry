#include "Menu.hpp"
#include "Recommender.hpp"
#include <iostream>
#include <limits>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/* ----------------- Helper IO ----------------- */

// Safely prompt for an integer in [min, max].
int getInt(const std::string &prompt, int min, int max) {
    int value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= min && value <= max) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return value;
        } else {
            std::cout << "Invalid input! Try again.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
}

// Prompt a yes/no question; returns true for y/Y.
bool getYesNo(const std::string &prompt) {
    char c;
    while (true) {
        std::cout << prompt << " (y/n): ";
        if (std::cin >> c) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (c == 'y' || c == 'Y') return true;
            if (c == 'n' || c == 'N') return false;
        }
        std::cout << "Please enter y or n.\n";
    }
}

// Safe getline (consumes leftover newline if present).
std::string safeGetline(const std::string &prompt) {
    std::string input;
    std::cout << prompt;
    std::getline(std::cin, input);
    // If empty due to previous extraction leaving newline, try once more
    if (input.empty()) {
        // Do NOT force a repeat; return empty string if user actually pressed enter.
    }
    return input;
}

/* ----------------- Utility to convert menu.json taste object -> array ----------------- */
static std::array<double, 5> tasteFromJson(const json &tasteObj) {
    // JSON keys: sweet, sour, bitter, salty, savory
    std::array<double,5> taste = {0,0,0,0,0};
    if (tasteObj.is_object()) {
        taste[0] = tasteObj.value("sweet", 0.0);
        taste[1] = tasteObj.value("sour", 0.0);
        taste[2] = tasteObj.value("bitter", 0.0);
        taste[3] = tasteObj.value("salty", 0.0);
        taste[4] = tasteObj.value("savory", 0.0);
    }
    return taste;
}

// Small helper to capitalize a word (used to form the user key)
static std::string capitalizeLocal(const std::string &input) {
    if (input.empty()) return input;
    std::string s = input;
    s[0] = static_cast<char>(std::toupper(s[0]));
    for (size_t i = 1; i < s.size(); ++i) s[i] = static_cast<char>(std::tolower(s[i]));
    return s;
}

/* ----------------- Main ----------------- */
int main() {
    std::cout << "Welcome to the Restaurant Bot!\n";
    std::cout << "I will help you create your perfect menu.\n";

    // Load master menu from provided menu.json
    std::vector<menu::MenuItem*> master; // will own pointers here until program end
    {
        std::ifstream mfile("menu.json");
        if (!mfile.is_open()) {
            std::cerr << "Failed to open master menu file 'menu.json'. Please ensure it exists.\n";
        } else {
            json masterJson;
            mfile >> masterJson;
            mfile.close();

            // For each category, create appropriate objects with default extra attributes
            if (masterJson.contains("starters") && masterJson["starters"].is_array()) {
                for (const auto &it : masterJson["starters"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::Starter(it.value("name",""), it.value("price",0.0), taste, false));
                }
            }
            if (masterJson.contains("salads") && masterJson["salads"].is_array()) {
                for (const auto &it : masterJson["salads"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::Salad(it.value("name",""), it.value("price",0.0), taste, false));
                }
            }
            if (masterJson.contains("main_courses") && masterJson["main_courses"].is_array()) {
                for (const auto &it : masterJson["main_courses"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::MainCourse(it.value("name",""), it.value("price",0.0), taste, false));
                }
            }
            if (masterJson.contains("drinks") && masterJson["drinks"].is_array()) {
                for (const auto &it : masterJson["drinks"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::Drink(it.value("name",""), it.value("price",0.0), taste, false, false));
                }
            }
            if (masterJson.contains("appetizers") && masterJson["appetizers"].is_array()) {
                for (const auto &it : masterJson["appetizers"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::Appetizer(it.value("name",""), it.value("price",0.0), taste, "before"));
                }
            }
            if (masterJson.contains("desserts") && masterJson["desserts"].is_array()) {
                for (const auto &it : masterJson["desserts"]) {
                    auto taste = tasteFromJson(it.value("taste_balance", json::object()));
                    master.push_back(new menu::Dessert(it.value("name",""), it.value("price",0.0), taste, false));
                }
            }
            std::cout << "Master menu loaded with " << master.size() << " items.\n";
        }
    }

    // Create user and recommender
    menu::User user;

    // --- NEW: Multi-user identification ---
    std::string inputFirst = safeGetline("Enter your first name: ");
    std::string inputLast = safeGetline("Enter your surname: ");
    std::string firstCap = capitalizeLocal(inputFirst);
    std::string lastCap = capitalizeLocal(inputLast);
    std::string fullKey = firstCap + " " + lastCap;

    // Load existing user_data.json (if any)
    json allData;
    std::ifstream ufile("user_data.json");
    if (ufile.is_open()) {
        try {
            ufile >> allData;
        } catch (...) {
            // malformed file, start fresh
            allData = json::object();
        }
        ufile.close();
    } else {
        allData = json::object();
    }

    // ensure users mapping exists and is an object
    if (!allData.contains("users") || !allData["users"].is_object()) {
        allData["users"] = json::object();
    }

    json &usersObj = allData["users"];

    if (usersObj.contains(fullKey)) {
        // found an existing user record
        if (getYesNo("Found previous data for " + fullKey + ". Reload your previous menu?")) {
            json rec = usersObj[fullKey];
            if (rec.contains("user")) {
                user.fromJson(rec["user"]);
            } else {
                // populate with first/last if missing
                json tmp;
                tmp["firstName"] = firstCap;
                tmp["lastName"] = lastCap;
                tmp["gender"] = "";
                user.fromJson(tmp);
            }
            if (rec.contains("menu")) {
                // write menu part to temporary JSON and let Menu::loadFromFile parse it
                std::ofstream tmp("tmp_user_menu.json");
                json mj;
                mj["items"] = rec["menu"];
                tmp << mj.dump(4);
                tmp.close();
                user.getMenu().loadFromFile("tmp_user_menu.json");
                std::remove("tmp_user_menu.json");
            }
        } else {
            // start fresh for this user (overwrite later on save)
            // set user's name fields so save uses them
            user.setFirstName(firstCap);
            user.setLastName(lastCap);
            // ask gender now (reuse the same validation logic as setUserInfo but avoid re-asking names)
            while (true) {
                std::string genderInput = safeGetline("Enter gender (Mr or Mrs): ");
                std::string genderLower = genderInput;
                std::transform(genderLower.begin(), genderLower.end(), genderLower.begin(), [](unsigned char c){ return std::tolower(c); });
                if (genderLower == "mr" || genderLower == "mrs") {
                    genderInput[0] = static_cast<char>(std::toupper(genderInput[0]));
                    user.setGender(genderInput);
                    break;
                }
                std::cout << "Invalid input. Please enter 'Mr' or 'Mrs' only.\n";
            }
            // menu remains empty (new)
        }
    } else {
        // new user — set names and ask remaining info via setUserInfo but avoid re-asking names
        user.setFirstName(firstCap);
        user.setLastName(lastCap);
        // ask gender similarly
        while (true) {
            std::string genderInput = safeGetline("Enter gender (Mr or Mrs): ");
            std::string genderLower = genderInput;
            std::transform(genderLower.begin(), genderLower.end(), genderLower.begin(), [](unsigned char c){ return std::tolower(c); });
            if (genderLower == "mr" || genderLower == "mrs") {
                genderInput[0] = static_cast<char>(std::toupper(genderInput[0]));
                user.setGender(genderInput);
                break;
            }
            std::cout << "Invalid input. Please enter 'Mr' or 'Mrs' only.\n";
        }
    }

    user.showUserInfo();

    ai::Recommender recommender;
    recommender.loadWeights();

    menu::Menu &menu = user.getMenu();

    bool running = true;
    while (running) {
        std::cout << "\n--- Menu Options ---\n";
        std::cout << "1. Browse master menu and add item\n";
        std::cout << "2. Add custom item\n";
        std::cout << "3. Remove item from your menu\n";
        std::cout << "4. Show your current menu\n";
        std::cout << "5. Predict satisfaction (AI) for an item\n";
        std::cout << "6. Save user data now\n";
        std::cout << "7. Exit\n";
        int choice = getInt("Choice: ", 1, 7);

        switch (choice) {
            case 1: {
                // Show master menu with indices
                if (master.empty()) {
                    std::cout << "Master menu is empty (menu.json missing or invalid).\n";
                    break;
                }
                std::cout << "\nMaster Menu (pick by number):\n";
                for (size_t i = 0; i < master.size(); ++i) {
                    std::cout << i + 1 << ". ";
                    master[i]->printInfo();
                }
                int idx = getInt("Enter number to add to your menu (0 to cancel): ", 0, static_cast<int>(master.size()));
                if (idx == 0) break;
                // Clone the chosen item into user's menu (preserve base data but default extras)
                auto* chosen = master[idx - 1];
                // Create a copy by using runtime type name checks (via dynamic_cast)
                if (auto s = dynamic_cast<menu::Starter*>(chosen)) {
                    bool hot = getYesNo("Serve hot?");
                    menu.addItem(new menu::Starter(s->getName(), s->getPrice(), s->getTaste(), hot));
                } else if (auto s = dynamic_cast<menu::Salad*>(chosen)) {
                    bool top = getYesNo("Add topping (+2.25)?");
                    double price = s->getPrice() + (top ? 2.25 : 0.0);
                    menu.addItem(new menu::Salad(s->getName(), price, s->getTaste(), top));
                } else if (auto m = dynamic_cast<menu::MainCourse*>(chosen)) {
                    bool veg = getYesNo("Vegetarian?");
                    menu.addItem(new menu::MainCourse(m->getName(), m->getPrice(), m->getTaste(), veg));
                } else if (auto d = dynamic_cast<menu::Drink*>(chosen)) {
                    bool carb = getYesNo("Carbonated? (+0.5)");
                    bool alc = getYesNo("Alcohol shot? (+2.5)");
                    double price = d->getPrice() + (carb ? 0.5 : 0.0) + (alc ? 2.5 : 0.0);
                    menu.addItem(new menu::Drink(d->getName(), price, d->getTaste(), carb, alc));
                } else if (auto a = dynamic_cast<menu::Appetizer*>(chosen)) {
                    std::string when = safeGetline("Serve time (before/after): ");
                    if (when.empty()) when = "before";
                    menu.addItem(new menu::Appetizer(a->getName(), a->getPrice(), a->getTaste(), when));
                } else if (auto ds = dynamic_cast<menu::Dessert*>(chosen)) {
                    bool choc = getYesNo("Add extra chocolate? (+1.5)");
                    double price = ds->getPrice() + (choc ? 1.5 : 0.0);
                    menu.addItem(new menu::Dessert(ds->getName(), price, ds->getTaste(), choc));
                }
                std::cout << "Item added to your menu.\n";
                break;
            }

            case 2: {
                // Add custom item by prompting for type and fields (keeps original approach but consistent taste order)
                std::cout << "\nCustom Item Types:\n1. Starter\n2. Salad\n3. Main Course\n4. Drink\n5. Appetizer\n6. Dessert\n";
                int type = getInt("Type: ", 1, 6);

                std::string name = safeGetline("Enter name: ");
                double price;
                std::cout << "Enter base price: ";
                while (!(std::cin >> price)) {
                    std::cout << "Please enter a valid number: ";
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                std::array<double, 5> taste = {0, 0, 0, 0, 0};
                std::cout << "Enter 5 taste ratings (0–10) in order: sweet, sour, bitter, salty, savory\n";
                for (int i = 0; i < 5; ++i) {
                    std::cout << "Taste " << i + 1 << ": ";
                    while (!(std::cin >> taste[i])) {
                        std::cout << "Please enter a valid number: ";
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    }
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                menu::MenuItem* item = nullptr;
                if (type == 1) {
                    bool hot = getYesNo("Hot?");
                    item = new menu::Starter(name, price, taste, hot);
                } else if (type == 2) {
                    bool topping = getYesNo("Add topping? (+2.25)");
                    if (topping) price += 2.25;
                    item = new menu::Salad(name, price, taste, topping);
                } else if (type == 3) {
                    bool veg = getYesNo("Vegetarian?");
                    item = new menu::MainCourse(name, price, taste, veg);
                } else if (type == 4) {
                    bool carb = getYesNo("Carbonated? (+0.5)");
                    if (carb) price += 0.5;
                    bool alc = getYesNo("Alcoholic shot? (+2.5)");
                    if (alc) price += 2.5;
                    item = new menu::Drink(name, price, taste, carb, alc);
                } else if (type == 5) {
                    std::string time = safeGetline("Serve time (before/after): ");
                    if (time.empty()) time = "before";
                    item = new menu::Appetizer(name, price, taste, time);
                } else if (type == 6) {
                    bool choc = getYesNo("Extra chocolate? (+1.5)");
                    if (choc) price += 1.5;
                    item = new menu::Dessert(name, price, taste, choc);
                }

                if (item != nullptr) {
                    menu.addItem(item);
                    std::cout << "Custom item added successfully.\n";
                }
                break;
            }

            case 3: {
                menu.showMenu();
                if (menu.getItemCount() == 0) break;
                int idx = getInt("Enter item number to remove: ", 1, menu.getItemCount());
                menu.removeItem(idx - 1);
                break;
            }

            case 4:
                menu.showMenu();
                break;

            case 5: {
                menu.showMenu();
                if (menu.getItemCount() == 0) {
                    std::cout << "Add some items first!\n";
                    break;
                }
                int idx = getInt("Select item number for AI prediction: ", 1, menu.getItemCount());
                auto taste = menu.getItemTaste(idx - 1);
                double pred = recommender.predict(taste);
                std::cout << "AI Predicted Satisfaction (0-10): " << pred << "\n";

                if (getYesNo("Would you like to give feedback to improve AI?")) {
                    double actual;
                    std::cout << "Enter your actual satisfaction (0–10): ";
                    while (!(std::cin >> actual) || actual < 0.0 || actual > 10.0) {
                        std::cout << "Please enter a number between 0 and 10: ";
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    }
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    recommender.update(taste, actual);
                    recommender.saveWeights();
                    std::cout << "Thanks! AI model updated.\n";
                }
                break;
            }

            case 6: {
                // Save this user's data inside allData["users"][fullKey]
                json record;
                record["user"] = user.toJson();

                // menu.saveToFile writes a different JSON format; build menu array directly
                json menuItems = json::array();
                // Workaround: save to a temp file then read it back
                menu.saveToFile("tmp_save_menu.json");
                {
                    std::ifstream t("tmp_save_menu.json");
                    if (t.is_open()) {
                        json mj;
                        t >> mj;
                        t.close();
                        if (mj.contains("items")) menuItems = mj["items"];
                    }
                    std::remove("tmp_save_menu.json");
                }
                record["menu"] = menuItems;

                // place into the users object under this user's fullKey
                usersObj[fullKey] = record;

                // write back the file
                std::ofstream f("user_data.json");
                f << allData.dump(4);
                f.close();
                std::cout << "User data saved to user_data.json\n";
                break;
            }

            case 7: {
                // On exit prompt to save
                if (getYesNo("Do you want to save your user data before exiting?")) {
                    json record;
                    record["user"] = user.toJson();
                    json menuItems = json::array();
                    menu.saveToFile("tmp_save_menu.json");
                    {
                        std::ifstream t("tmp_save_menu.json");
                        if (t.is_open()) {
                            json mj;
                            t >> mj;
                            t.close();
                            if (mj.contains("items")) menuItems = mj["items"];
                        }
                        std::remove("tmp_save_menu.json");
                    }
                    record["menu"] = menuItems;
                    usersObj[fullKey] = record;
                    std::ofstream f("user_data.json");
                    f << allData.dump(4);
                    f.close();
                    std::cout << "User data saved to user_data.json\n";
                }
                running = false;
                break;
            }

            default:
                break;
        }
    }

    // Cleanup master pointers
    for (auto* p : master) delete p;

    std::cout << "\nGoodbye!\n";
    return 0;
}
