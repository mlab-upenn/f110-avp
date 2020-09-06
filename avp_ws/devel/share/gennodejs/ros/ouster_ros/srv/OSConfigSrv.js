// Auto-generated. Do not edit!

// (in-package ouster_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class OSConfigSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OSConfigSrvRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OSConfigSrvRequest
    let len;
    let data = new OSConfigSrvRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/OSConfigSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OSConfigSrvRequest(null);
    return resolved;
    }
};

class OSConfigSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.metadata = null;
    }
    else {
      if (initObj.hasOwnProperty('metadata')) {
        this.metadata = initObj.metadata
      }
      else {
        this.metadata = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OSConfigSrvResponse
    // Serialize message field [metadata]
    bufferOffset = _serializer.string(obj.metadata, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OSConfigSrvResponse
    let len;
    let data = new OSConfigSrvResponse(null);
    // Deserialize message field [metadata]
    data.metadata = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.metadata.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/OSConfigSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd37888e5a47bef783c189dec5351027e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string metadata
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OSConfigSrvResponse(null);
    if (msg.metadata !== undefined) {
      resolved.metadata = msg.metadata;
    }
    else {
      resolved.metadata = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: OSConfigSrvRequest,
  Response: OSConfigSrvResponse,
  md5sum() { return 'd37888e5a47bef783c189dec5351027e'; },
  datatype() { return 'ouster_ros/OSConfigSrv'; }
};
