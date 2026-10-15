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

class SetAwbRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.rgain = null;
      this.bgain = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('rgain')) {
        this.rgain = initObj.rgain
      }
      else {
        this.rgain = 0.0;
      }
      if (initObj.hasOwnProperty('bgain')) {
        this.bgain = initObj.bgain
      }
      else {
        this.bgain = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAwbRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [rgain]
    bufferOffset = _serializer.float32(obj.rgain, buffer, bufferOffset);
    // Serialize message field [bgain]
    bufferOffset = _serializer.float32(obj.bgain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAwbRequest
    let len;
    let data = new SetAwbRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rgain]
    data.rgain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bgain]
    data.bgain = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/SetAwbRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b05e71e59f5c8aef36d8f8a6684dd851';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Set AWB (auto white balance) mode and (in manual mode) R/B gains.
    #
    # mode == 0 (AUTO)   : sends opcode 0x31 only; rgain and bgain ignored.
    # mode == 1 (MANUAL) : sends opcode 0x32 then 0x33 (rgain, bgain).
    #                     Gr/Gb are fixed to 1.0 by the device.
    #
    # Valid manual ranges:
    #   rgain : 0.1 .. 4.0
    #   bgain : 0.1 .. 4.0
    uint8 mode
    float32 rgain
    float32 bgain
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAwbRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.rgain !== undefined) {
      resolved.rgain = msg.rgain;
    }
    else {
      resolved.rgain = 0.0
    }

    if (msg.bgain !== undefined) {
      resolved.bgain = msg.bgain;
    }
    else {
      resolved.bgain = 0.0
    }

    return resolved;
    }
};

class SetAwbResponse {
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
    // Serializes a message object of type SetAwbResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [rc]
    bufferOffset = _serializer.int32(obj.rc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAwbResponse
    let len;
    let data = new SetAwbResponse(null);
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
    return 'odin_ros_driver/SetAwbResponse';
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
    const resolved = new SetAwbResponse(null);
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
  Request: SetAwbRequest,
  Response: SetAwbResponse,
  md5sum() { return 'b1976cc70ace03d84cebe2d1a9bc817e'; },
  datatype() { return 'odin_ros_driver/SetAwb'; }
};
