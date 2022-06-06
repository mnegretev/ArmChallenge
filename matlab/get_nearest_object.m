function [obj_position, obj_class] = get_nearest_object(sub_point_cloud, sub_current_q, robot)
    % REQUIRES STATISTICS AND MACHINE LEARNING TOOLBOX
    
    %Get current end effector position
    Q = receive(sub_current_q, 3);
    Q = Q.Position(2:8);
    T = getTransform(robot, Q', 'EndEffector_Link', 'base_link');
    p  = [T(1,4) T(2,4) T(3,4)];
    
    %clusterize and classify by color
    [xyz, rgb] = get_cloud_wrt_base(sub_point_cloud, sub_current_q, robot);
    [xyz, rgb] = filter_cloud(xyz, rgb);
    [clusters_xyz, clusters_rgb, centroids] = clusterize_cloud(xyz, rgb);
    min_d = inf;
    obj_position = [inf inf inf];
    obj_class = 'unknown';
    for i=1:length(clusters_xyz)
        colors = clusterize_color(clusters_xyz{i}, clusters_rgb{i});
        obj_class = classify_by_color(colors);
        if ~strcmp(obj_class, 'unknown')
            d = norm(centroids(i,:) - p);
            if d < min_d
                min_d = d;
                obj_position = centroids(i,:);
            end
        end
    end
    