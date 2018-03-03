/*
  Copyright (c) 2018, Michael McCoy
*/
#include <csignal>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "peripherals.h"
#include <iostream>
#include <cmath>
#include <ncurses.h>

using namespace peripherals;


double max(double a, double b) {
  return a > b ? a : b;
}

double min(double a, double b) {
  return a < b ? a : b;
}


/**
 * Frequency tick for a given frequency
 */
double freqTick(double freqMHz) {
  return pow(10.0, int(log10(freqMHz)) - 2.0);
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
  return round(max(4.0 - log10(freqMHz), 0.));
}


void clearLine(int offset = 0) {
    int x,y;
    getyx(stdscr, y, x);
    move(y + offset,0);
    clrtoeol();
}

int main(int argc, char** argv) {
  Peripherals& peripherals = Peripherals::getInstance();

  // PLog documentation at https://github.com/SergiusTheBest/plog
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::debug, &consoleAppender);

  /*
  // TODO:
  // Add options for setting frequency, pin (12,13,18,19), and duty cycle
  // Allow arrow keys to update frequency, duty cycle
  LOG_DEBUG << "Starting test";
  pwmTest(peripherals);
  LOG_DEBUG << "Test finished";
  */

  double freqMHz = 100.0;
  double dutyCycle = 0.5;

  constexpr char EXIT_CHAR = 'x';
  static const char help[] =
    "Use left/right to change frequency, up/down to change duty cycle, 'x' to quit.\n";

  int ch = int(EXIT_CHAR);

  setlocale(LC_ALL, "");

  initscr();/* Start curses mode */
  raw();/* Line buffering disabled*/
  keypad(stdscr, TRUE);/* Get arrows, shifts, etc..*/
  noecho();/* Don't echo() while we do getch */

  printw(help);

  do {
    clearLine();
    printw("%%%.1f duty @ %.*fMHz", 100.0*dutyCycle, sigFig(freqMHz), freqMHz);
    ch = getch();
    switch (ch) {
    case KEY_UP:
      dutyCycle = min(1.0, dutyCycle + dutyTick(dutyCycle));
      break;
    case KEY_DOWN:
      dutyCycle = max(0.0, dutyCycle - dutyTick(dutyCycle));
      break;
    case KEY_LEFT:
      freqMHz -= freqTick(freqMHz);
      break;
    case KEY_RIGHT:
      freqMHz += freqTick(freqMHz);
      break;
    default:
      printw(help);
      break;
    }
  } while (ch != int(EXIT_CHAR));
  endwin();/* End curses mode  */
  return 0;
}
