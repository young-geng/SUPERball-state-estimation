function saveVideo(fname,frames,framerate)
    v = VideoWriter(fname);
    
    v.FrameRate = framerate;
    v.open();
    for i = 1:size(frames,2)
        v.writeVideo(frames{i});
    end
    v.close();
end