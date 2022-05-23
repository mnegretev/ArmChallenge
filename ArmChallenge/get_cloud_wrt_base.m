function [xyz, rgb] = get_cloud_wrt_base(sub_cloud, sub_current_pose, robot)
    %%% Get current articular position
    Q_current = receive(sub_current_pose, 1);
    Q_current = Q_current.Position(2:8);
    
    %%% Get  Transform from base link to camera link
    base_to_bracelet = getTransform(robot, Q_current', 'Bracelet_Link', 'base_link');
    bracelet_to_cam  = [ 1.0 0.0 0.0 0.0; 0.0 1.0 0.0 -0.055; 0.0 0.0  1.0 -0.055; 0 0 0 1];
    cam_to_camera    = [-1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0000; 0.0 0.0 -1.0 0.1000; 0 0 0 1];
    transform = base_to_bracelet*bracelet_to_cam*cam_to_camera;
    
    %%% Get last point cloud and transform to base link
    cloud = receive(sub_cloud, 1);
    xyz = rosReadXYZ(cloud);
    rgb = rosReadRGB(cloud);
    for i=1:length(xyz)
        p = (transform*[xyz(i,:)'; 1])';
        c = rgb(i,:);
        rgb(i,:) = [c(3) c(2) c(1)];
        xyz(i,:) = p(1:3);
    end
    
    