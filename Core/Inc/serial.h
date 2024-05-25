/*
 * serial.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Cosmin
 */

#ifndef SERIAL_DLD_H
#define SERIAL_DLD_H

#ifdef UART_SERIAL_ENABLE

#include "state_machine.h"
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void ParseCommand(const char* cmd, const char* arg);    // forward declaration, defined in main.cpp


class SerialAdapter {
public:
  static void begin() {
    // Already configured in Setup().
    // Serial.begin(115200);
  }
  static bool Connected() { return true; }
  static bool AlwaysConnected() { return false; }
  static UartSerial& stream() { return Serial; }
  static const char* response_header() { return ""; }
  static const char* response_footer() { return ""; }
};


// Command-line parser. Easiest way to use it is to start the arduino
// serial monitor.
template<class SA> /* SA = Serial Adapter */
class Parser : StateMachine {
public:
  Parser(){}
  const char* name() { return "Parser"; }

  void Setup() {
    SA::begin();
  }

  void Loop() {

    STATE_MACHINE_BEGIN();
    while (true) {
      while (!SA::Connected()) YIELD();
      if (!SA::AlwaysConnected()) {
        // STDOUT << "Welcome to ProffieOS " << version << ". Type 'help' for more info.\n";
      }

      while (SA::Connected()) {
        while (!SA::stream().available()) YIELD();
        int c = SA::stream().read();
        if (c < 0) { break; }
//        LL_mDelay(19);
        if (c == '\n' || c == '\r') {
          if (cmd_) ParseLine();
          len_ = 0;
          space_ = 0;
          free(cmd_);
          cmd_ = nullptr;
          continue;
        }
        if (len_ + 1 >= space_) {
          int new_space = space_ * 3 / 2 + 8;
          char* tmp = (char*)realloc(cmd_, new_space);
          if (tmp) {
            space_ = new_space;
            cmd_ = tmp;
          } else {
//            STDOUT.println("Line too long.");
            len_ = 0;
            space_ = 0;
            free(cmd_);
            cmd_ = nullptr;
            continue;
          }
        }
        cmd_[len_] = c;
        cmd_[len_ + 1] = 0;
        len_++;
      }
      len_ = 0;
      space_ = 0;
      free(cmd_);
      cmd_ = nullptr;
    }
    STATE_MACHINE_END();
  }

  void ParseLine() {
    if (len_ == 0) return;
    while (len_ > 0 && (cmd_[len_-1] == '\r' || cmd_[len_-1] == ' ')) {
      len_--;
      cmd_[len_] = 0;
    }
    if (cmd_[0] == '#') {
//      STDOUT.println(cmd_);
      return;
    }
    // stdout_output = &SA::stream();
//    STDOUT.print(SA::response_header());
    char *cmd = cmd_;
    while (*cmd == ' ') cmd++;
    char *e = cmd;
    while (*e != ' ' && *e) e++;
    if (*e) {
      *e = 0;
      e++;  // e is now argument (if any)
    } else {
      e = nullptr;
    }

  ParseCommand(cmd, e);   // call main parser

  }

private:
  int len_ = 0;
  char* cmd_ = nullptr;
  int space_ = 0;

};

Parser<SerialAdapter> parser;

#endif // UART_SERIAL_ENABLE

#endif
