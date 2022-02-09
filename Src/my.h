void Flash_Write (uint32_t Flash_Address, uint32_t Flash_Data);
void SetBrightness(int channel, int brightness);
void SetRealBrightness(int ch, double bn);
double getFadeStep(int fromBright, int toBright, int period_ms);
void LedOn_FadeOnTime(int pwmChannel, int brightness, int fadePeriod_ms);
void LedOn_FadeOnTime60(int pwmChannel, int fadePeriod_ms);
void LedOn_FadeOffTime60(int pwmChannel,int fadePeriod_ms);
void LedOn_FadeOnTime2(int pwmChannel, int fadePeriod_ms);
void LedOn_FadeOnTime3(int pwmChannel, int fadePeriod_ms);
void LedControlProc(void); //10ms
void keyProc(void);
