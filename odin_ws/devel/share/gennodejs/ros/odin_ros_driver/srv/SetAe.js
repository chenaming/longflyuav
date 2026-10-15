// Auto-generated. Do not edit!

// (in-package odin_ros_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetAeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.exposure_time = null;
      this.gain = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('exposure_time')) {
        this.exposure_time = initObj.exposure_time
      }
      else {
        this.exposure_time = 0.0;
      }
      if (initObj.hasOwnProperty('gain')) {
        this.gain = initObj.gain
      }
      else {
        this.gain = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAeRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [exposure_time]
    bufferOffset = _serializer.float32(obj.exposure_time, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _serializer.float32(obj.gain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAeRequest
    let len;
    let data = new SetAeRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [exposure_time]
    data.exposure_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain]
    data.gain = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/SetAeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2124bfa3e5e0fcf89af46aebdb5cdd4e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Set AE (auto exposure) mode and (in manual mode) exposure / gain.
    #
    # mode == 0 (AUTO)   : sends opcode 0x02 only; exposure_time and gain ignored.
    # mode == 1 (MANUAL) : sends opcode 0x03 then 0x06 (exposure_time, gain).
    #
    # Valid manual ranges (device-enforced; out-of-range returns rc=403):
    #   exposure_time : 0.0001 s .. 0.033 s
    #   gain          : 1.0     .. 64.0
    uint8 mode
    float32 exposure_time
    float32 gain
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAeRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.exposure_time !== undefined) {
      resolved.exposure_time = msg.exposure_time;
    }
    else {
      resolved.exposure_time = 0.0
    }

    if (msg.gain !== undefined) {
      resolved.gain = msg.gain;
    }
    else {
      resolved.gain = 0.0
    }

    return resolved;
    }
};

class SetAeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.rc = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('rc')) {
        this.rc = initObj.rc
      }
      else {
        this.rc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [rc]
    bufferOffset = _serializer.int32(obj.rc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAeResponse
    let len;
    let data = new SetAeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rc]
    data.rc = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/SetAeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '430062776294a8265666428b4e2a6e92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAeResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.rc !== undefined) {
      resolved.rc = msg.rc;
    }
    else {
      resolved.rc = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetAeRequest,
  Response: SetAeResponse,
  md5sum() { return 'a79a03de733d9c62ec6c16126efd1953'; },
  datatype() { return 'odin_ros_driver/SetAe'; }
};
