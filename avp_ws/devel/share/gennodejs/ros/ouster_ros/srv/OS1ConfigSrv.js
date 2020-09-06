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

class OS1ConfigSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OS1ConfigSrvRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OS1ConfigSrvRequest
    let len;
    let data = new OS1ConfigSrvRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/OS1ConfigSrvRequest';
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
    const resolved = new OS1ConfigSrvRequest(null);
    return resolved;
    }
};

class OS1ConfigSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hostname = null;
      this.lidar_mode = null;
      this.beam_azimuth_angles = null;
      this.beam_altitude_angles = null;
      this.imu_to_sensor_transform = null;
      this.lidar_to_sensor_transform = null;
    }
    else {
      if (initObj.hasOwnProperty('hostname')) {
        this.hostname = initObj.hostname
      }
      else {
        this.hostname = '';
      }
      if (initObj.hasOwnProperty('lidar_mode')) {
        this.lidar_mode = initObj.lidar_mode
      }
      else {
        this.lidar_mode = '';
      }
      if (initObj.hasOwnProperty('beam_azimuth_angles')) {
        this.beam_azimuth_angles = initObj.beam_azimuth_angles
      }
      else {
        this.beam_azimuth_angles = [];
      }
      if (initObj.hasOwnProperty('beam_altitude_angles')) {
        this.beam_altitude_angles = initObj.beam_altitude_angles
      }
      else {
        this.beam_altitude_angles = [];
      }
      if (initObj.hasOwnProperty('imu_to_sensor_transform')) {
        this.imu_to_sensor_transform = initObj.imu_to_sensor_transform
      }
      else {
        this.imu_to_sensor_transform = [];
      }
      if (initObj.hasOwnProperty('lidar_to_sensor_transform')) {
        this.lidar_to_sensor_transform = initObj.lidar_to_sensor_transform
      }
      else {
        this.lidar_to_sensor_transform = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OS1ConfigSrvResponse
    // Serialize message field [hostname]
    bufferOffset = _serializer.string(obj.hostname, buffer, bufferOffset);
    // Serialize message field [lidar_mode]
    bufferOffset = _serializer.string(obj.lidar_mode, buffer, bufferOffset);
    // Serialize message field [beam_azimuth_angles]
    bufferOffset = _arraySerializer.float64(obj.beam_azimuth_angles, buffer, bufferOffset, null);
    // Serialize message field [beam_altitude_angles]
    bufferOffset = _arraySerializer.float64(obj.beam_altitude_angles, buffer, bufferOffset, null);
    // Serialize message field [imu_to_sensor_transform]
    bufferOffset = _arraySerializer.float64(obj.imu_to_sensor_transform, buffer, bufferOffset, null);
    // Serialize message field [lidar_to_sensor_transform]
    bufferOffset = _arraySerializer.float64(obj.lidar_to_sensor_transform, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OS1ConfigSrvResponse
    let len;
    let data = new OS1ConfigSrvResponse(null);
    // Deserialize message field [hostname]
    data.hostname = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lidar_mode]
    data.lidar_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [beam_azimuth_angles]
    data.beam_azimuth_angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [beam_altitude_angles]
    data.beam_altitude_angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [imu_to_sensor_transform]
    data.imu_to_sensor_transform = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [lidar_to_sensor_transform]
    data.lidar_to_sensor_transform = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.hostname.length;
    length += object.lidar_mode.length;
    length += 8 * object.beam_azimuth_angles.length;
    length += 8 * object.beam_altitude_angles.length;
    length += 8 * object.imu_to_sensor_transform.length;
    length += 8 * object.lidar_to_sensor_transform.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/OS1ConfigSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb865a85fb3f39f43bf99730f10225b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string hostname
    string lidar_mode
    float64[] beam_azimuth_angles
    float64[] beam_altitude_angles
    float64[] imu_to_sensor_transform
    float64[] lidar_to_sensor_transform
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OS1ConfigSrvResponse(null);
    if (msg.hostname !== undefined) {
      resolved.hostname = msg.hostname;
    }
    else {
      resolved.hostname = ''
    }

    if (msg.lidar_mode !== undefined) {
      resolved.lidar_mode = msg.lidar_mode;
    }
    else {
      resolved.lidar_mode = ''
    }

    if (msg.beam_azimuth_angles !== undefined) {
      resolved.beam_azimuth_angles = msg.beam_azimuth_angles;
    }
    else {
      resolved.beam_azimuth_angles = []
    }

    if (msg.beam_altitude_angles !== undefined) {
      resolved.beam_altitude_angles = msg.beam_altitude_angles;
    }
    else {
      resolved.beam_altitude_angles = []
    }

    if (msg.imu_to_sensor_transform !== undefined) {
      resolved.imu_to_sensor_transform = msg.imu_to_sensor_transform;
    }
    else {
      resolved.imu_to_sensor_transform = []
    }

    if (msg.lidar_to_sensor_transform !== undefined) {
      resolved.lidar_to_sensor_transform = msg.lidar_to_sensor_transform;
    }
    else {
      resolved.lidar_to_sensor_transform = []
    }

    return resolved;
    }
};

module.exports = {
  Request: OS1ConfigSrvRequest,
  Response: OS1ConfigSrvResponse,
  md5sum() { return 'bb865a85fb3f39f43bf99730f10225b5'; },
  datatype() { return 'ouster_ros/OS1ConfigSrv'; }
};
