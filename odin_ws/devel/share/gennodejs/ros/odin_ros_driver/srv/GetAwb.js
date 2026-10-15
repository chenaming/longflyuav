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

class GetAwbRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAwbRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAwbRequest
    let len;
    let data = new GetAwbRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/GetAwbRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Query current AWB (auto white balance) state.
    # Request: empty.
    # Response: success + raw return code + decoded fields.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAwbRequest(null);
    return resolved;
    }
};

class GetAwbResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.rc = null;
      this.rgain = null;
      this.grgain = null;
      this.gbgain = null;
      this.bgain = null;
      this.cct = null;
      this.ccri = null;
      this.is_converged = null;
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
      if (initObj.hasOwnProperty('rgain')) {
        this.rgain = initObj.rgain
      }
      else {
        this.rgain = 0.0;
      }
      if (initObj.hasOwnProperty('grgain')) {
        this.grgain = initObj.grgain
      }
      else {
        this.grgain = 0.0;
      }
      if (initObj.hasOwnProperty('gbgain')) {
        this.gbgain = initObj.gbgain
      }
      else {
        this.gbgain = 0.0;
      }
      if (initObj.hasOwnProperty('bgain')) {
        this.bgain = initObj.bgain
      }
      else {
        this.bgain = 0.0;
      }
      if (initObj.hasOwnProperty('cct')) {
        this.cct = initObj.cct
      }
      else {
        this.cct = 0.0;
      }
      if (initObj.hasOwnProperty('ccri')) {
        this.ccri = initObj.ccri
      }
      else {
        this.ccri = 0.0;
      }
      if (initObj.hasOwnProperty('is_converged')) {
        this.is_converged = initObj.is_converged
      }
      else {
        this.is_converged = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAwbResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [rc]
    bufferOffset = _serializer.int32(obj.rc, buffer, bufferOffset);
    // Serialize message field [rgain]
    bufferOffset = _serializer.float32(obj.rgain, buffer, bufferOffset);
    // Serialize message field [grgain]
    bufferOffset = _serializer.float32(obj.grgain, buffer, bufferOffset);
    // Serialize message field [gbgain]
    bufferOffset = _serializer.float32(obj.gbgain, buffer, bufferOffset);
    // Serialize message field [bgain]
    bufferOffset = _serializer.float32(obj.bgain, buffer, bufferOffset);
    // Serialize message field [cct]
    bufferOffset = _serializer.float32(obj.cct, buffer, bufferOffset);
    // Serialize message field [ccri]
    bufferOffset = _serializer.float32(obj.ccri, buffer, bufferOffset);
    // Serialize message field [is_converged]
    bufferOffset = _serializer.uint8(obj.is_converged, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAwbResponse
    let len;
    let data = new GetAwbResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rc]
    data.rc = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rgain]
    data.rgain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grgain]
    data.grgain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gbgain]
    data.gbgain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bgain]
    data.bgain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cct]
    data.cct = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ccri]
    data.ccri = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_converged]
    data.is_converged = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 30;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/GetAwbResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '508f4cd8169f95ac5caf4673a24ce5b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success            # true if rc == 0
    int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
    float32 rgain
    float32 grgain
    float32 gbgain
    float32 bgain
    float32 cct             # color temperature in Kelvin
    float32 ccri            # color temperature deviation
    uint8 is_converged      # 1=converged, 0=not converged
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAwbResponse(null);
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

    if (msg.rgain !== undefined) {
      resolved.rgain = msg.rgain;
    }
    else {
      resolved.rgain = 0.0
    }

    if (msg.grgain !== undefined) {
      resolved.grgain = msg.grgain;
    }
    else {
      resolved.grgain = 0.0
    }

    if (msg.gbgain !== undefined) {
      resolved.gbgain = msg.gbgain;
    }
    else {
      resolved.gbgain = 0.0
    }

    if (msg.bgain !== undefined) {
      resolved.bgain = msg.bgain;
    }
    else {
      resolved.bgain = 0.0
    }

    if (msg.cct !== undefined) {
      resolved.cct = msg.cct;
    }
    else {
      resolved.cct = 0.0
    }

    if (msg.ccri !== undefined) {
      resolved.ccri = msg.ccri;
    }
    else {
      resolved.ccri = 0.0
    }

    if (msg.is_converged !== undefined) {
      resolved.is_converged = msg.is_converged;
    }
    else {
      resolved.is_converged = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAwbRequest,
  Response: GetAwbResponse,
  md5sum() { return '508f4cd8169f95ac5caf4673a24ce5b2'; },
  datatype() { return 'odin_ros_driver/GetAwb'; }
};
