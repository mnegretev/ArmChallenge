function [filtered_xyz, filtered_rgb] = filter_cloud(xyz, rgb)
    counter = 1;
    filtered_xyz = [];
    filtered_rgb = [];
    for i=1:length(xyz)
        hsv = rgb2hsv(rgb(i,:));
        if hsv(1) > 27/360 && hsv(1) < 33/360
            continue
        end
        if xyz(i,3) > 0.02
            filtered_xyz(counter,:) = xyz(i,:);
            filtered_rgb(counter,:) = rgb(i,:);
            counter = counter + 1;
        end
    end