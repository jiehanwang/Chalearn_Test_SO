function dist = dtw_fast(tt,rr)
nn = size(tt,1);
mm = size(rr,1);
% % ÷°∆•≈‰æ‡¿Îæÿ’Û
% % d = zeros(n,m);
% gap = 1;
% for i = 1:gap:n
%     for j = 1:gap:m
%         d(floor(i/gap)+1,floor(j/gap)+1) = sum((t(i,:)-r(j,:)).^2);
%     end
% end

gap = 1;
for i=1:gap:nn
    t(floor(i/gap)+1,:) = tt(i,:);
end
for i=1:gap:mm
    r(floor(i/gap)+1,:) = rr(i,:);
end

n = size(t,1);
m = size(r,1);

% d = pdist2(tt(1:gap:end,:),rr(1:gap:end,:));

d_tr = t*r';
d_t = repmat(diag(t*t'),1,m);
d_r = repmat(diag(r*r'),1,n)';

d=(d_t + d_r - 2*d_tr);

[p,q,D,sc] = dpfast(d);
dist = D(size(D,1),size(D,2));

%% show the path
% imagesc(d);
% colormap(1-gray);
% hold on;
% plot(q,p,'r');
% hold off;

