function [filtered_xyz, filtered_rgb] = filter_cloud(xyz, rgb)
    counter = 1;
    filtered_xyz = [];
    filtered_rgb = [];
    for i=1:length(xyz)
        hsv = rgb2hsv(rgb(i,:));
        if hsv(1) > 24/360 && hsv(1) < 50/360
            continue
        end
        if hsv(1) > 37/360 && hsv(1) < 44/360
            continue
        end
        if xyz(i,3) > 0.02
            filtered_xyz(counter,:) = xyz(i,:);
            filtered_rgb(counter,:) = rgb(i,:);
            counter = counter + 1;
        end
    end