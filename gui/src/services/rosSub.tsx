import RosLib from "roslib";

export const useRosClient = () => {
    const rosClient = new RosLib.Ros({
        url: "ws://localhost:9090"
    })
    const cameraSub = new RosLib.ActionClient({
        ros: rosClient,
        serverName: "/camera_action",
        actionName: "camera_msgs/CameraAction"
    });
    return { rosClient, cameraSub };
}


