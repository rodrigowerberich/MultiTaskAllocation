#include "MainOptions.h"

#include <iostream>

using namespace std;

MainOptions::MainOptions(int argc, char* argv[]) :
        argc_(argc),
        argv_(argv) {
    appName_ = argv_[0];
    this->parse();
}

MainOptions::~MainOptions() {
}

std::string MainOptions::getAppName() const {
    return appName_;
}

void MainOptions::parse() {
    typedef pair<string, string> Option;
    Option* option = new pair<string, string>();
    for (const char* const * i = this->begin() + 1; i != this->end(); i++) {
        const string p = *i;
        if (option->first == "" && p[0] == '-') {
            option->first = p;
            if (i == this->last()) {
                options_.insert(Option(option->first, option->second));
            }
            continue;
        } else if (option->first != "" && p[0] == '-') {
            option->second = "null"; /* or leave empty? */
            options_.insert(Option(option->first, option->second));
            option->first = p;
            option->second = "";
            if (i == this->last()) {
                options_.insert(Option(option->first, option->second));
            }
            continue;
        } else if (option->first != "") {
            option->second = p;
            options_.insert(Option(option->first, option->second));
            option->first = "";
            option->second = "";
            continue;
        }
    }
}

void MainOptions::printOptions() const {
    std::map<std::string, std::string>::const_iterator m = options_.begin();
    int i = 0;
    if (options_.empty()) {
        cout << "No parameters\n";
    }
    for (; m != options_.end(); m++, ++i) {
        cout << "Parameter [" << i << "] [" << (*m).first << " " << (*m).second
                << "]\n";
    }
}

const char* const *MainOptions::begin() const {
    return argv_;
}

const char* const *MainOptions::end() const {
    return argv_ + argc_;
}

const char* const *MainOptions::last() const {
    return argv_ + argc_ - 1;
}

bool MainOptions::hasKey(const std::string& key) const {
    return options_.find(key) != options_.end();
}

MainOptions::Option* MainOptions::getParamFromKey(
        const std::string& key) const {
    const Options::const_iterator i = options_.find(key);
    MainOptions::Option* o = 0;
    if (i != options_.end()) {
        o = new MainOptions::Option((*i).first, (*i).second);
    }
    return o;
}