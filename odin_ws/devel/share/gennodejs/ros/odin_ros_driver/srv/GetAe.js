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

class GetAeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAeRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAeRequest
    let len;
    let data = new GetAeRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/GetAeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Query current AE (auto exposure) state.
    # Request: empty.
    # Response: success + raw return code + decoded fields.
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAeRequest(null);
    return resolved;
    }
};

class GetAeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.rc = null;
      this.exposure_time = null;
      this.gain = null;
      this.iso = null;
      this.brightness = null;
      this.is_converged = null;
      this.env_lv = null;
      this.fps = null;
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
      if (initObj.hasOwnProperty('iso')) {
        this.iso = initObj.iso
      }
      else {
        this.iso = 0;
      }
      if (initObj.hasOwnProperty('brightness')) {
        this.brightness = initObj.brightness
      }
      else {
        this.brightness = 0.0;
      }
      if (initObj.hasOwnProperty('is_converged')) {
        this.is_converged = initObj.is_converged
      }
      else {
        this.is_converged = 0;
      }
      if (initObj.hasOwnProperty('env_lv')) {
        this.env_lv = initObj.env_lv
      }
      else {
        this.env_lv = 0.0;
      }
      if (initObj.hasOwnProperty('fps')) {
        this.fps = initObj.fps
      }
      else {
        this.fps = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [rc]
    bufferOffset = _serializer.int32(obj.rc, buffer, bufferOffset);
    // Serialize message field [exposure_time]
    bufferOffset = _serializer.float32(obj.exposure_time, buffer, bufferOffset);
    // Serialize message field [gain]
    bufferOffset = _serializer.float32(obj.gain, buffer, bufferOffset);
    // Serialize message field [iso]
    bufferOffset = _serializer.int32(obj.iso, buffer, bufferOffset);
    // Serialize message field [brightness]
    bufferOffset = _serializer.float32(obj.brightness, buffer, bufferOffset);
    // Serialize message field [is_converged]
    bufferOffset = _serializer.uint8(obj.is_converged, buffer, bufferOffset);
    // Serialize message field [env_lv]
    bufferOffset = _serializer.float32(obj.env_lv, buffer, bufferOffset);
    // Serialize message field [fps]
    bufferOffset = _serializer.float32(obj.fps, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAeResponse
    let len;
    let data = new GetAeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rc]
    data.rc = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [exposure_time]
    data.exposure_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain]
    data.gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [iso]
    data.iso = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [brightness]
    data.brightness = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_converged]
    data.is_converged = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [env_lv]
    data.env_lv = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fps]
    data.fps = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 30;
  }

  static datatype() {
    // Returns string type for a service object
    return 'odin_ros_driver/GetAeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '542c4f6644aaf51ceda9dea371da93c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success            # true if rc == 0
    int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
    float32 exposure_time   # seconds (manual range 0.0001 .. 0.033)
    float32 gain            # analog gain
    int32 iso               # equivalent ISO
    float32 brightness      # average frame brightness
    uint8 is_converged      # 1=converged, 0=not converged
    float32 env_lv          # ambient luminance level
    float32 fps             # current frame rate
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAeResponse(null);
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

    if (msg.iso !== undefined) {
      resolved.iso = msg.iso;
    }
    else {
      resolved.iso = 0
    }

    if (msg.brightness !== undefined) {
      resolved.brightness = msg.brightness;
    }
    else {
      resolved.brightness = 0.0
    }

    if (msg.is_converged !== undefined) {
      resolved.is_converged = msg.is_converged;
    }
    else {
      resolved.is_converged = 0
    }

    if (msg.env_lv !== undefined) {
      resolved.env_lv = msg.env_lv;
    }
    else {
      resolved.env_lv = 0.0
    }

    if (msg.fps !== undefined) {
      resolved.fps = msg.fps;
    }
    else {
      resolved.fps = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAeRequest,
  Response: GetAeResponse,
  md5sum() { return '542c4f6644aaf51ceda9dea371da93c3'; },
  datatype() { return 'odin_ros_driver/GetAe'; }
};
