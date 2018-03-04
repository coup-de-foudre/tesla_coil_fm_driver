/*
  Copyright (c) 2018, Michael McCoy
*/
#include "pwm.h"

#include <ncurses.h>

#include <csignal>
#include <cmath>

#include <algorithm>
#include <iostream>

#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"


using namespace pwm;


/**
 * Frequency tick for a given frequency
 */
double freqTick(double freqMHz) {
  return pow(10.0, int(log10(freqMHz / 2.0)) - 2.0);
}


/**
 * Duty cycle tick
 */
double dutyTick(double dutyCycle) {
  return 0.01;
}


/**
 * Significant figures needed for frequency
 */
int sigFig(double freqMHz) {
  return round(std::max(4.0 - log10(freqMHz), 0.));
}


void clearLine(int offset = 0) {
    int x,y;
    getyx(stdscr, y, x);
    move(y + offset, 0);
    clrtoeol();
}

int main(int argc, char** argv) {
  Peripherals& peripherals = Peripherals::getInstance();

  // PLog documentation at https://github.com/SergiusTheBest/plog
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::debug, &consoleAppender);

  constexpr char EXIT_CHAR = 'q';
  double freqMHz = 100.0;
  double dutyCycle = 0.5;

  bool showUsage = false;
  for (int i = 1; i < argc; i++) {
    if (std::string("-f") == argv[i]) {
      freqMHz = std::atof(argv[i+1]);
      i++;
    } else {
      showUsage = true;
    }
  }
  if (showUsage) {
    std::cout << "Usage: " << argv[0] << " [-f freqMHz=100.0]" << std::endl;
    return 0;
  }

  static const char help[] =
    "Use left/right to change frequency, up/down to change duty cycle, 'q' to quit.\n";

  PwmController pwmController(freqMHz, dutyCycle);
  int ch = int(EXIT_CHAR);

  setlocale(LC_ALL, "");

  initscr(); // Start curses mode
  raw();  // Line buffering disabled
  keypad(stdscr, TRUE);  // Get arrows, shifts, etc..
  noecho();  // Don't echo() while we do getch

  printw(help);

  do {
    pwmController.setTargetFreqMHz(freqMHz);
    pwmController.setTargetDutyCycle(dutyCycle);

    double actualFreqMHz = pwmController.getHardwareFreqMHz();
    double actualDutyCycle = pwmController.getHardwareDutyCycle();

    printw("ACTUAL: %%%.1f duty @ %.*fMHz\n",
	   100.0*actualDutyCycle, sigFig(actualFreqMHz), actualFreqMHz);
    printw("TARGET: %%%.1f duty @ %.*fMHz\n",
	   100.0*dutyCycle, sigFig(freqMHz), freqMHz);

    ch = getch();
    switch (ch) {
    case KEY_UP:
      dutyCycle = std::min(1.0, dutyCycle + dutyTick(dutyCycle));
      break;
    case KEY_DOWN:
      dutyCycle = std::max(0.0, dutyCycle - dutyTick(dutyCycle));
      break;
    case KEY_LEFT:
      freqMHz -= freqTick(freqMHz);
      break;
    case KEY_RIGHT:
      freqMHz += freqTick(freqMHz);
      break;
    default:
      break;
    }
    clearLine(-1);
    clearLine(-1);
  } while (ch != int(EXIT_CHAR));
  endwin();/* End curses mode  */
  return 0;
}
