// Auto-generated. Do not edit!

// (in-package main.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ArmAngles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arm0_joint0 = null;
      this.arm0_joint1 = null;
      this.arm1_joint0 = null;
      this.arm1_joint1 = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('arm0_joint0')) {
        this.arm0_joint0 = initObj.arm0_joint0
      }
      else {
        this.arm0_joint0 = 0.0;
      }
      if (initObj.hasOwnProperty('arm0_joint1')) {
        this.arm0_joint1 = initObj.arm0_joint1
      }
      else {
        this.arm0_joint1 = 0.0;
      }
      if (initObj.hasOwnProperty('arm1_joint0')) {
        this.arm1_joint0 = initObj.arm1_joint0
      }
      else {
        this.arm1_joint0 = 0.0;
      }
      if (initObj.hasOwnProperty('arm1_joint1')) {
        this.arm1_joint1 = initObj.arm1_joint1
      }
      else {
        this.arm1_joint1 = 0.0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmAngles
    // Serialize message field [arm0_joint0]
    bufferOffset = _serializer.float32(obj.arm0_joint0, buffer, bufferOffset);
    // Serialize message field [arm0_joint1]
    bufferOffset = _serializer.float32(obj.arm0_joint1, buffer, bufferOffset);
    // Serialize message field [arm1_joint0]
    bufferOffset = _serializer.float32(obj.arm1_joint0, buffer, bufferOffset);
    // Serialize message field [arm1_joint1]
    bufferOffset = _serializer.float32(obj.arm1_joint1, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmAngles
    let len;
    let data = new ArmAngles(null);
    // Deserialize message field [arm0_joint0]
    data.arm0_joint0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [arm0_joint1]
    data.arm0_joint1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [arm1_joint0]
    data.arm1_joint0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [arm1_joint1]
    data.arm1_joint1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'main/ArmAngles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03c786ea5bb5b9ff9771f16c1a88ee81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 arm0_joint0
    float32 arm0_joint1
    float32 arm1_joint0
    float32 arm1_joint1
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmAngles(null);
    if (msg.arm0_joint0 !== undefined) {
      resolved.arm0_joint0 = msg.arm0_joint0;
    }
    else {
      resolved.arm0_joint0 = 0.0
    }

    if (msg.arm0_joint1 !== undefined) {
      resolved.arm0_joint1 = msg.arm0_joint1;
    }
    else {
      resolved.arm0_joint1 = 0.0
    }

    if (msg.arm1_joint0 !== undefined) {
      resolved.arm1_joint0 = msg.arm1_joint0;
    }
    else {
      resolved.arm1_joint0 = 0.0
    }

    if (msg.arm1_joint1 !== undefined) {
      resolved.arm1_joint1 = msg.arm1_joint1;
    }
    else {
      resolved.arm1_joint1 = 0.0
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = ArmAngles;
