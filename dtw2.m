function dist = dtw2(tt,rr)
% n = size(t,1);
% m = size(r,1);
% % Ö¡Æ¥Åä¾àÀë¾ØÕó
% d = zeros(n,m);
% for i = 1:n
%     for j = 1:m
%         d(i,j) = sum((t(i,:)-r(j,:)).^2);
% %         d(i,j) = sqrt(sum((t(i,:)-r(j,:)).^2));
%     end
% end
%%
nn = size(tt,1);
mm = size(rr,1);

gap = 1;
for i=1:gap:nn
    t(floor(i/gap)+1,:) = tt(i,:);
end
for i=1:gap:mm
    r(floor(i/gap)+1,:) = rr(i,:);
end

n = size(t,1);
m = size(r,1);

d_tr = t*r';
d_t = repmat(diag(t*t'),1,m);
d_r = repmat(diag(r*r'),1,n)';

d=(d_t + d_r - 2*d_tr);
%%
% ÀÛ»ı¾àÀë¾ØÕó
D = ones(n,m) * realmax;
D(1,1) = d(1,1);
% ¶¯Ì¬¹æ»®
for i = 2:n
    for j = 1:m
        D1 = D(i-1,j);
        if j>1
            D2 = D(i-1,j-1);
        else
            D2 = realmax;
        end
        if j>2
            D3 = D(i-1,j-2);
        else
            D3 = realmax;
        end
        D(i,j) = d(i,j) + min([D1,D2,D3]);
    end
end
dist = D(n,m); 