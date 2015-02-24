// The MIT License (MIT)

// Copyright (c) 2015 Miquel Massot

// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef CONSOLE_COLOR_H
#define CONSOLE_COLOR_H

#include <iostream>
#include <string>

enum ColorCode {
  NOCOLOR = -1,
  BLACK = 0,
  RED = 1,
  GREEN = 2,
  YELLOW = 3,
  BLUE = 4,
  MAGENTA = 5,
  CYAN = 6,
  WHITE = 7
};

/**
 * @brief Class ConsoleColor
 * @details Instantiate a variable of the class with one or two color codes.
 * The first represents the foreground color and the second the background.
 *
 * @param fg Foreground Color Code
 * @param bg Background Color Code
 */
class ConsoleColor {
 public:
  ConsoleColor(int fg, int bg)
    : fg_(fg), bg_(bg) {}
  ConsoleColor(int fg)
    : fg_(fg), bg_(NOCOLOR) {}

  std::string operator() (const std::string& text) {
    std::ostringstream oss;
    if (bg_ == NOCOLOR) {
      oss << "\33[" << 30+fg_ << "m" << text << "\33[0m";
    } else {
      oss << "\33[" << 30+fg_ << ";" << 40+bg_ << "m" << text << "\33[0m";
    }
    return oss.str();
  }
 private:
  int fg_, bg_;
};

#endif // CONSOLE_COLOR_H