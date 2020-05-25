
void X12begin( unsigned int _steps, unsigned char _pinStep, unsigned char _pinDir, unsigned char _pinReset);

void setupTimer(uint8_t _pin);
void noToneX12(uint8_t _pin);
int advance();

void X12setPosition(unsigned int pos);
unsigned int X12getPosition();
void X12full();
void X12zero();

