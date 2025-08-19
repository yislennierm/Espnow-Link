namespace ghst {
  constexpr uint32_t BAUD = 420000;
  constexpr uint32_t CFG  = SERIAL_8N1;
  constexpr bool RX_INVERT = false;
  constexpr bool TX_INVERT = true;

  bool isCompleteFrame(const uint8_t *buf, int len);
  void debug_decode_channels(const uint8_t *frame);
}
