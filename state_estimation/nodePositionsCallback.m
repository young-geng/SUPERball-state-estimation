function nodePositionsCallback(data)
    global nodepos
    nodepos = reshape(data,[3 12])';
end