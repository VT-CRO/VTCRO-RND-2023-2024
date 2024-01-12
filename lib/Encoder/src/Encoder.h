#ifndef ENCODER_H
#define ENCODER_H

#include "Subject.hpp"

class Encoder : public Subject<int> {
public:
  Encoder();
  Encoder(Encoder &&) = default;
  Encoder(const Encoder &) = default;
  Encoder &operator=(Encoder &&) = default;
  Encoder &operator=(const Encoder &) = default;
  ~Encoder();

  void init();

private:
  struct Encoder_props {};

  typedef struct Encoder_props Encoder_props;

  Encoder_props props;
  int count;
};

#endif // !ENCODER_H
