function recordVideo(framerate)
    global Frames;
    Frames = {};
    i = 0;
    while (1)
        i = i + 1;
%         hardcopy(gcf,sprintf('plots/%d.ps',i),'-dpsc2','-append');
        f = getframe(gcf);%screencapture(gcf);
        Frames{i} = f.cdata;
        %savefig(sprintf('plots/%d.fig',i));
        %pause(1./framerate);
        i
    end
end