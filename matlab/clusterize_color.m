function centroids = clusterize_color(xyz, rgb)
    nrgb = zeros(size(rgb));
    for i=1:length(rgb)
        nrgb(i,:) = rgb(i,:)/max(rgb(i,:));
    end

    K = 15;
    [idx, centroids] = kmeans(nrgb, K);
    d = min(pdist(centroids));
    while d < 0.5 && K > 0
        K = K - 1;
        [idx, centroids] = kmeans(nrgb, K);
        if K > 1
            d = min(pdist(centroids));
        else
            d = inf;
        end
    end
    %rows = size(centroids);
    %rows = rows(1);
    %for i=1:rows
    %    centroids(i,:) = centroids(i,:)/max(centroids(i,:));
    %end
    %scatter3(xyz(:,1), xyz(:,2), xyz(:,3), 20, rgb);
    
    