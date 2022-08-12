function [ rmse, R, t, scale ] = AlignSimEfficient( gtPos, estimatedPos )


centroid_A = mean(estimatedPos,1);
centroid_B = mean(gtPos,1);

N = size(estimatedPos,1);
H = (estimatedPos - repmat(centroid_A, N, 1))' * (gtPos - repmat(centroid_B, N, 1));
[U,S,V] = svd(H);
R = V*U';
if det(R) < 0
    V(:,3) = V(:,3) * -1;
    R = V*U';
end

mR_cA = -R*(centroid_A)';

A = estimatedPos * R' + repmat(mR_cA', N, 1);
B = gtPos - repmat(centroid_B, N, 1);

saa = 0;
sab = 0;
for i=1:size(A,1)
    saa = saa + A(i,:)*A(i,:)';
    sab = sab + A(i,:)*B(i,:)';
end

scale = sab/saa;
t = scale*mR_cA + centroid_B';
rmse =  (sum(sum((scale*A-B).^2))/size(A,1)).^0.5;

if(isnan(scale))
    R = nan(3);
end


end

