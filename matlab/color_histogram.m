function H = color_histogram(rgb)
    H = zeros(64, 1);
    for i=1:length(rgb)
        %nRGB = uint8(255*rgb(i,:)/max(rgb(i,:))); %Gets normalized RGB
        %idx  = bitsra(nRGB(1), 6)*16 + bitsra(nRGB(2), 6)*4 + bitsra(nRGB(3), 6);
        nRGB = rgb(i,:)/max(rgb(i,:)); %Gets normalized RGB
        idx = uint8(nRGB(1)*48 + nRGB(2)*12 + nRGB(3)*3) + 1;
        H(idx) = H(idx) + 1;
    end
    H = H/max(H);
    bar(H);