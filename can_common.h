#ifndef _CAN_COMMON_
#define _CAN_COMMON_

#include <Arduino.h>

/** Define the typical baudrate for CAN communication. */

#define CAN_BPS_500K	500000


#define SIZE_LISTENERS	4 //number of classes that can register as listeners with this class
#define CAN_DEFAULT_BAUD	500000

//some defines to allow older API to still work
#define attachMBHandler setCallback
#define detachMBHandler removeCallback
#define attachGeneralHandler setGeneralHandler
#define detachGeneralHandler removeGeneralHandler

class BitRef
{
public:
    BitRef& operator=( bool x )
    {
        *byteRef = (*byteRef & ~(1 << bitPos));
        if (x) *byteRef = *byteRef | (1 << bitPos);
        return *this;
    }
    //BitRef& operator=( const BitRef& x );

    operator bool() const 
    {
        if (*byteRef & (1 << bitPos)) return true;
        return false;
    }
public:
    BitRef(uint8_t *ref, int pos)
    {
        byteRef = ref;
        bitPos = pos;
    }
private:
    uint8_t *byteRef;
    int bitPos;
};

typedef union {
    uint64_t uint64;
    uint32_t uint32[2]; 
    uint16_t uint16[4];
    uint8_t  uint8[8];
    int64_t int64;
    int32_t int32[2]; 
    int16_t int16[4];
    int8_t  int8[8];

    //deprecated names used by older code
    uint64_t value;
    struct {
        uint32_t low;
        uint32_t high;
    };
    struct {
        uint16_t s0;
        uint16_t s1;
        uint16_t s2;
        uint16_t s3;
    };
    uint8_t bytes[8];
    uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
    struct {
        uint8_t bitField[8];
        const bool operator[]( int pos ) const
        {
            if (pos < 0 || pos > 63) return 0;
            int bitFieldIdx = pos / 8;
            return (bitField[bitFieldIdx] >> pos) & 1;
        }
        BitRef operator[]( int pos )
        {
            if (pos < 0 || pos > 63) return BitRef((uint8_t *)&bitField[0], 0);
            uint8_t *ptr = (uint8_t *)&bitField[0]; 
            return BitRef(ptr + (pos / 8), pos & 7);
        }
    } bit;
} BytesUnion;

typedef union {
    uint64_t uint64[8];
    uint32_t uint32[16]; 
    uint16_t uint16[32];
    uint8_t  uint8[64];
    int64_t int64[8];
    int32_t int32[16]; 
    int16_t int16[32];
    int8_t  int8[64];

    struct {
        uint8_t bitField[64];
        const bool operator[]( int pos ) const
        {
            if (pos < 0 || pos > 511) return 0; //64 8 bit bytes is 512 bits, we start counting bits at 0
            int bitfieldIdx = pos / 8;
            return (bitField[bitfieldIdx] >> pos) & 1;
        }
        BitRef operator[]( int pos )
        {
            if (pos < 0 || pos > 511) return BitRef((uint8_t *)&bitField[0], 0);
            uint8_t *ptr = (uint8_t *)&bitField[0]; 
            return BitRef(ptr + (pos / 8), pos & 7);
        }
    } bit;
} BytesUnion_FD;

class CAN_FRAME
{
public:
    CAN_FRAME();

    BytesUnion data;    // 64 bits - lots of ways to access it.
    uint32_t id;        // 29 bit if ide set, 11 bit otherwise
    uint32_t fid;       // family ID - used internally to library
    uint32_t timestamp; // CAN timer value when mailbox message was received.
    uint8_t rtr;        // Remote Transmission Request (1 = RTR, 0 = data frame)
    uint8_t priority;   // Priority but only important for TX frames and then only for special uses (0-31)
    uint8_t extended;   // Extended ID flag
    uint8_t length;     // Number of data bytes
    
};

class CANListener
{
public:
  CANListener();

  virtual void gotFrame(CAN_FRAME *frame, int mailbox);

  void setCallback(uint8_t mailBox);
  void removeCallback(uint8_t mailBox);
  void setGeneralHandler();
  void removeGeneralHandler();
  void initialize();
  bool isCallbackActive(int callback);
  void setNumFilters(int numFilt);

private:
  uint32_t callbacksActive; //bitfield letting the code know which callbacks to actually try to use (for object oriented callbacks only)
  bool generalCBActive; //is the general callback registered?
  int numFilters; //filters, mailboxes, whichever, how many do we have?
};

/*Abstract function that mostly just sets an interface that all descendants must implement */
class CAN_COMMON
{
public:

    CAN_COMMON(int numFilt);

    //Public API that needs to be re-implemented by subclasses
	virtual int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) = 0;
    virtual int _setFilter(uint32_t id, uint32_t mask, bool extended) = 0;
	virtual uint32_t init(uint32_t ul_baudrate) = 0;
    virtual uint32_t beginAutoSpeed() = 0;
    virtual uint32_t set_baudrate(uint32_t ul_baudrate) = 0;
    virtual void setListenOnlyMode(bool state) = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual bool sendFrame(CAN_FRAME& txFrame) = 0;
	virtual bool rx_avail() = 0;
	virtual uint16_t available() = 0; //like rx_avail but returns the number of waiting frames
	virtual uint32_t get_rx_buff(CAN_FRAME &msg) = 0;

    //Public API common to all subclasses - don't need to be re-implemented
    //wrapper for syntactic sugar reasons
    //note to my dumb self - functions cannot be both virtual and have multiple versions where the parameter list is different
    //you have to pick one or the other.
	inline uint32_t read(CAN_FRAME &msg) { return get_rx_buff(msg); }    
    int watchFor(); //allow anything through
	int watchFor(uint32_t id); //allow just this ID through (automatic determination of extended status)
    int watchFor(uint32_t id, uint32_t mask); //allow a range of ids through
    int watchFor(uint32_t id, uint32_t mask, bool ext);
	int watchForRange(uint32_t id1, uint32_t id2); //try to allow the range from id1 to id2 - automatically determine base ID and mask
	uint32_t begin();
	uint32_t begin(uint32_t baudrate);
    uint32_t begin(uint32_t baudrate, uint8_t enPin);
	uint32_t getBusSpeed();
	int setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    int setRXFilter(uint32_t id, uint32_t mask, bool extended);
    boolean attachObj(CANListener *listener);
	boolean detachObj(CANListener *listener);
    void setGeneralCallback( void (*cb)(CAN_FRAME *) );
	void setCallback(uint8_t mailbox, void (*cb)(CAN_FRAME *));
    void removeCallback();
    void removeCallback(uint8_t mailbox);
    void removeGeneralCallback();
    void attachCANInterrupt( void (*cb)(CAN_FRAME *) ) {setGeneralCallback(cb);}
	void attachCANInterrupt(uint8_t mailBox, void (*cb)(CAN_FRAME *));
	void detachCANInterrupt(uint8_t mailBox);
    bool isFaulted();
    bool hasRXFault();
    bool hasTXFault();
    void setDebuggingMode(bool mode);

    
    bool debuggingMode;

protected:
	CANListener *listener[SIZE_LISTENERS];
    void (*cbGeneral)(CAN_FRAME *); //general callback if no per-mailbox or per-filter entries matched
    void (*cbCANFrame[32])(CAN_FRAME *); //array of function pointers - disgusting syntax though.
    uint32_t busSpeed;
    int numFilters;
    int enablePin;
    bool faulted;
    bool rxFault;
    bool txFault;    
};

#endif

