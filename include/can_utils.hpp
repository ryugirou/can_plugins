  #include <boost/array.hpp>
  #define CAN_MTU 8
  template<typename T>
    union _Encapsulator
    {
      T data;
      uint64_t i;
    };

  template <typename T>
    void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
    {
      _Encapsulator<T> _e;

      for(int i = 0; i < sizeof(T); i++)
      {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
      }

      data = _e.data;
    }

  template<typename T>
    void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
    {
      _Encapsulator<T> _e;
      _e.data = data;

      for(int i = sizeof(T); i > 0;)
      {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
      }
    }

  