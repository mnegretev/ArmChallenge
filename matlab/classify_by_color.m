function obj_class = classify_by_color(colors)
    ref     = [0 0 1;    1 0 0; 0 1 0; 1 1 0];
    classes = {'bottle'; 'can'; 'can'; 'can'};
    %Check if blue is present, if so, then obj is a bottle
    obj_class = 'unknown';
    rows = size(colors);
    rows = rows(1);
    for c=1:length(ref)
        min_d = inf;    
        for i=1:rows
            d = norm(colors(i,:) - ref(c,:));
            if d < min_d
                min_d = d;
            end
        end
        if min_d < 0.35
            obj_class = classes{c};
            return
        end
    end
    