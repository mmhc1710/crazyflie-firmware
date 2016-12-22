#define DEBUG_MODULE "HelloDeck"
#include "debug.h"

#include "deck.h"


static void helloInit()
{
  DEBUG_PRINT("Hello Crazyflie 2.0 deck world!\n");
}

static bool helloTest()
{
  DEBUG_PRINT("Hello test passed!\n");
  return true;
}

static const DeckDriver helloDriver = {
  .name = "myHello",
  .init = helloInit,
  .test = helloTest,
};

DECK_DRIVER(helloDriver);
