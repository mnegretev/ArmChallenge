function [obj_position, obj_class, obj_axis] = get_nearest_object(sub_point_cloud, sub_current_q, robot)
    % REQUIRES STATISTICS AND MACHINE LEARNING TOOLBOX
    
    %Get current end effector position
    Q = receive(sub_current_q, 3);
    Q = Q.Position(2:8);
    T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
    p  = [T(1,4) T(2,4) T(3,4)];
    
    %Transform point cloud wrt base_link
    [xyz, rgb] = get_cloud_wrt_base(sub_point_cloud, sub_current_q, robot);
    %Display transformed cloud
    figure(1)
    pcshow(pointCloud(xyz, 'Color', uint8(rgb*255)));
    title("Original Point Cloud")
    %Remove table from cloud
    [xyz, rgb] = filter_cloud(xyz, rgb);
    %Display cloud without table
    figure(2)
    pcshow(pointCloud(xyz, 'Color', uint8(rgb*255)));
    title("Object clusters")
    hold on
    %Clusterize cloud
    [clusters_xyz, clusters_rgb, centroids] = clusterize_cloud(xyz, rgb);
    %Display centroids
    scatter3(centroids(:,1), centroids(:,2), centroids(:,3), 200, "red", 'filled', 'o');
    hold off
    %Find the nearest recognized object
    min_d = inf;
    obj_position = [inf inf inf];
    obj_class = 'unknown';
    obj_idx = 0;
    for i=1:length(clusters_xyz)
        colors = clusterize_color(clusters_xyz{i}, clusters_rgb{i});
        obj_class_temp = classify_by_color(colors);
        if ~strcmp(obj_class_temp, 'unknown')
            d = norm(centroids(i,:) - p);
            if d < min_d
                min_d = d;
                obj_position = centroids(i,:);
                obj_class = obj_class_temp;
                obj_idx = i;
            end
        end
    end
    if obj_idx == 0
        obj_axis = zeros(3,3);
        return
    end
    %Correct object position 
    theta = atan2(obj_position(2), obj_position(1));
    rho   = sqrt(obj_position(2)^2 + obj_position(1)^2);
    rho   = rho + 0.05;
    obj_position(1) = rho*cos(theta);
    obj_position(2) = rho*sin(theta);
    %Display nearest known object's cloud
    figure(3)
    pcshow(pointCloud(clusters_xyz{obj_idx}, 'Color', uint8(255*clusters_rgb{obj_idx})));
    title("Recognized: "+obj_class);
    %Calculates object-oriented axis by pca
    obj_axis = pca(clusters_xyz{obj_idx});
    