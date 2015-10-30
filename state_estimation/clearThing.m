function clearThing(thing)
global hvid;
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
thing.NewMessageFcn = [];
% Close the video object. This is important! The file may not play properly if you don't close it.
close(hvid);
end

