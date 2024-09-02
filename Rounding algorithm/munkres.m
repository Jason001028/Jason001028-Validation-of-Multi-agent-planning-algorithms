function assignment = munkres(costMat)
    [m, n] = size(costMat);
    assignment = zeros(m, n);
    costMat(costMat == inf) = 1e+9;
    
    % Step 1: Subtract row minima
    costMat = costMat - min(costMat, [], 2);
    
    % Step 2: Subtract column minima
    costMat = costMat - min(costMat, [], 1);
    
    % Step 3: Cover all zeros with a minimum number of lines
    coveredRows = false(m, 1);
    coveredCols = false(1, n);
    starZ = false(m, n);
    primeZ = false(m, n);
    
    for i = 1:m
        for j = 1:n
            if costMat(i,j) == 0 && ~coveredRows(i) && ~coveredCols(j)
                starZ(i,j) = true;
                coveredRows(i) = true;
                coveredCols(j) = true;
            end
        end
    end
    
    while sum(coveredRows) + sum(coveredCols) < m
        % Step 4: Find an uncovered zero
        zeroRow = 0;
        zeroCol = 0;
        for i = 1:m
            for j = 1:n
                if costMat(i,j) == 0 && ~coveredRows(i) && ~coveredCols(j)
                    zeroRow = i;
                    zeroCol = j;
                    break;
                end
            end
            if zeroRow ~= 0
                break;
            end
        end
        
        if zeroRow == 0
            % Step 5: Find the smallest uncovered value
            minVal = min(min(costMat(~coveredRows, ~coveredCols)));
            costMat(~coveredRows, :) = costMat(~coveredRows, :) - minVal;
            costMat(:, coveredCols) = costMat(:, coveredCols) + minVal;
        else
            primeZ(zeroRow, zeroCol) = true;
            starCol = find(starZ(zeroRow, :), 1);
            if isempty(starCol)
                % Step 6: Augment path
                while true
                    starZ(zeroRow, zeroCol) = true;
                    primeZ(zeroRow, zeroCol) = false;
                    starRow = find(starZ(:, zeroCol), 1);
                    if isempty(starRow)
                        break;
                    end
                    zeroRow = starRow;
                    zeroCol = find(primeZ(zeroRow, :), 1);
                end
                coveredRows(:) = false;
                coveredCols(:) = false;
                starZ(primeZ) = true;
                primeZ(:) = false;
            else
                coveredRows(zeroRow) = true;
                coveredCols(starCol) = false;
            end
        end
    end
    
    assignment = starZ;
end