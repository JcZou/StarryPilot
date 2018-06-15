#pragma once
// MESSAGE ADRC PACKING

#define MAVLINK_MSG_ID_ADRC 666

MAVPACKED(
typedef struct __mavlink_adrc_t {
 float v; /*< setpoint signal.*/
 float v1; /*< TD output v1.*/
 float v2; /*< TD output v2.*/
 float z1; /*< z1 follows x1.*/
 float z2; /*< z2 follows x2.*/
 float z3; /*< z3 follows x3.*/
 float s1; /*< cascade ESO, s1.*/
 float s2; /*< cascade ESO, s2.*/
 float s3; /*< cascade ESO, s3.*/
}) mavlink_adrc_t;

#define MAVLINK_MSG_ID_ADRC_LEN 36
#define MAVLINK_MSG_ID_ADRC_MIN_LEN 36
#define MAVLINK_MSG_ID_666_LEN 36
#define MAVLINK_MSG_ID_666_MIN_LEN 36

#define MAVLINK_MSG_ID_ADRC_CRC 8
#define MAVLINK_MSG_ID_666_CRC 8



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADRC { \
    666, \
    "ADRC", \
    9, \
    {  { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_adrc_t, v) }, \
         { "v1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adrc_t, v1) }, \
         { "v2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adrc_t, v2) }, \
         { "z1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adrc_t, z1) }, \
         { "z2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adrc_t, z2) }, \
         { "z3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adrc_t, z3) }, \
         { "s1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adrc_t, s1) }, \
         { "s2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adrc_t, s2) }, \
         { "s3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adrc_t, s3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADRC { \
    "ADRC", \
    9, \
    {  { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_adrc_t, v) }, \
         { "v1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adrc_t, v1) }, \
         { "v2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adrc_t, v2) }, \
         { "z1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adrc_t, z1) }, \
         { "z2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adrc_t, z2) }, \
         { "z3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adrc_t, z3) }, \
         { "s1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adrc_t, s1) }, \
         { "s2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adrc_t, s2) }, \
         { "s3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adrc_t, s3) }, \
         } \
}
#endif

/**
 * @brief Pack a adrc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param v setpoint signal.
 * @param v1 TD output v1.
 * @param v2 TD output v2.
 * @param z1 z1 follows x1.
 * @param z2 z2 follows x2.
 * @param z3 z3 follows x3.
 * @param s1 cascade ESO, s1.
 * @param s2 cascade ESO, s2.
 * @param s3 cascade ESO, s3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adrc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float v, float v1, float v2, float z1, float z2, float z3, float s1, float s2, float s3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADRC_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, v1);
    _mav_put_float(buf, 8, v2);
    _mav_put_float(buf, 12, z1);
    _mav_put_float(buf, 16, z2);
    _mav_put_float(buf, 20, z3);
    _mav_put_float(buf, 24, s1);
    _mav_put_float(buf, 28, s2);
    _mav_put_float(buf, 32, s3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADRC_LEN);
#else
    mavlink_adrc_t packet;
    packet.v = v;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.z1 = z1;
    packet.z2 = z2;
    packet.z3 = z3;
    packet.s1 = s1;
    packet.s2 = s2;
    packet.s3 = s3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADRC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADRC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
}

/**
 * @brief Pack a adrc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param v setpoint signal.
 * @param v1 TD output v1.
 * @param v2 TD output v2.
 * @param z1 z1 follows x1.
 * @param z2 z2 follows x2.
 * @param z3 z3 follows x3.
 * @param s1 cascade ESO, s1.
 * @param s2 cascade ESO, s2.
 * @param s3 cascade ESO, s3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adrc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float v,float v1,float v2,float z1,float z2,float z3,float s1,float s2,float s3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADRC_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, v1);
    _mav_put_float(buf, 8, v2);
    _mav_put_float(buf, 12, z1);
    _mav_put_float(buf, 16, z2);
    _mav_put_float(buf, 20, z3);
    _mav_put_float(buf, 24, s1);
    _mav_put_float(buf, 28, s2);
    _mav_put_float(buf, 32, s3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADRC_LEN);
#else
    mavlink_adrc_t packet;
    packet.v = v;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.z1 = z1;
    packet.z2 = z2;
    packet.z3 = z3;
    packet.s1 = s1;
    packet.s2 = s2;
    packet.s3 = s3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADRC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADRC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
}

/**
 * @brief Encode a adrc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adrc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adrc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adrc_t* adrc)
{
    return mavlink_msg_adrc_pack(system_id, component_id, msg, adrc->v, adrc->v1, adrc->v2, adrc->z1, adrc->z2, adrc->z3, adrc->s1, adrc->s2, adrc->s3);
}

/**
 * @brief Encode a adrc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adrc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adrc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adrc_t* adrc)
{
    return mavlink_msg_adrc_pack_chan(system_id, component_id, chan, msg, adrc->v, adrc->v1, adrc->v2, adrc->z1, adrc->z2, adrc->z3, adrc->s1, adrc->s2, adrc->s3);
}

/**
 * @brief Send a adrc message
 * @param chan MAVLink channel to send the message
 *
 * @param v setpoint signal.
 * @param v1 TD output v1.
 * @param v2 TD output v2.
 * @param z1 z1 follows x1.
 * @param z2 z2 follows x2.
 * @param z3 z3 follows x3.
 * @param s1 cascade ESO, s1.
 * @param s2 cascade ESO, s2.
 * @param s3 cascade ESO, s3.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adrc_send(mavlink_channel_t chan, float v, float v1, float v2, float z1, float z2, float z3, float s1, float s2, float s3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADRC_LEN];
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, v1);
    _mav_put_float(buf, 8, v2);
    _mav_put_float(buf, 12, z1);
    _mav_put_float(buf, 16, z2);
    _mav_put_float(buf, 20, z3);
    _mav_put_float(buf, 24, s1);
    _mav_put_float(buf, 28, s2);
    _mav_put_float(buf, 32, s3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADRC, buf, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
#else
    mavlink_adrc_t packet;
    packet.v = v;
    packet.v1 = v1;
    packet.v2 = v2;
    packet.z1 = z1;
    packet.z2 = z2;
    packet.z3 = z3;
    packet.s1 = s1;
    packet.s2 = s2;
    packet.s3 = s3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADRC, (const char *)&packet, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
#endif
}

/**
 * @brief Send a adrc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adrc_send_struct(mavlink_channel_t chan, const mavlink_adrc_t* adrc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adrc_send(chan, adrc->v, adrc->v1, adrc->v2, adrc->z1, adrc->z2, adrc->z3, adrc->s1, adrc->s2, adrc->s3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADRC, (const char *)adrc, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADRC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adrc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float v, float v1, float v2, float z1, float z2, float z3, float s1, float s2, float s3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, v);
    _mav_put_float(buf, 4, v1);
    _mav_put_float(buf, 8, v2);
    _mav_put_float(buf, 12, z1);
    _mav_put_float(buf, 16, z2);
    _mav_put_float(buf, 20, z3);
    _mav_put_float(buf, 24, s1);
    _mav_put_float(buf, 28, s2);
    _mav_put_float(buf, 32, s3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADRC, buf, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
#else
    mavlink_adrc_t *packet = (mavlink_adrc_t *)msgbuf;
    packet->v = v;
    packet->v1 = v1;
    packet->v2 = v2;
    packet->z1 = z1;
    packet->z2 = z2;
    packet->z3 = z3;
    packet->s1 = s1;
    packet->s2 = s2;
    packet->s3 = s3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADRC, (const char *)packet, MAVLINK_MSG_ID_ADRC_MIN_LEN, MAVLINK_MSG_ID_ADRC_LEN, MAVLINK_MSG_ID_ADRC_CRC);
#endif
}
#endif

#endif

// MESSAGE ADRC UNPACKING


/**
 * @brief Get field v from adrc message
 *
 * @return setpoint signal.
 */
static inline float mavlink_msg_adrc_get_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field v1 from adrc message
 *
 * @return TD output v1.
 */
static inline float mavlink_msg_adrc_get_v1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field v2 from adrc message
 *
 * @return TD output v2.
 */
static inline float mavlink_msg_adrc_get_v2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z1 from adrc message
 *
 * @return z1 follows x1.
 */
static inline float mavlink_msg_adrc_get_z1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z2 from adrc message
 *
 * @return z2 follows x2.
 */
static inline float mavlink_msg_adrc_get_z2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z3 from adrc message
 *
 * @return z3 follows x3.
 */
static inline float mavlink_msg_adrc_get_z3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field s1 from adrc message
 *
 * @return cascade ESO, s1.
 */
static inline float mavlink_msg_adrc_get_s1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field s2 from adrc message
 *
 * @return cascade ESO, s2.
 */
static inline float mavlink_msg_adrc_get_s2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field s3 from adrc message
 *
 * @return cascade ESO, s3.
 */
static inline float mavlink_msg_adrc_get_s3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a adrc message into a struct
 *
 * @param msg The message to decode
 * @param adrc C-struct to decode the message contents into
 */
static inline void mavlink_msg_adrc_decode(const mavlink_message_t* msg, mavlink_adrc_t* adrc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adrc->v = mavlink_msg_adrc_get_v(msg);
    adrc->v1 = mavlink_msg_adrc_get_v1(msg);
    adrc->v2 = mavlink_msg_adrc_get_v2(msg);
    adrc->z1 = mavlink_msg_adrc_get_z1(msg);
    adrc->z2 = mavlink_msg_adrc_get_z2(msg);
    adrc->z3 = mavlink_msg_adrc_get_z3(msg);
    adrc->s1 = mavlink_msg_adrc_get_s1(msg);
    adrc->s2 = mavlink_msg_adrc_get_s2(msg);
    adrc->s3 = mavlink_msg_adrc_get_s3(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADRC_LEN? msg->len : MAVLINK_MSG_ID_ADRC_LEN;
        memset(adrc, 0, MAVLINK_MSG_ID_ADRC_LEN);
    memcpy(adrc, _MAV_PAYLOAD(msg), len);
#endif
}
