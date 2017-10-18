x = [1 2 3 4 0 0 0 0 0 0 0 9];
w = [1 0 0 0 .1 0 0 0 0 0 .2 1];
w = w / sum(w);
c = 0;
n = size(x, 2);
idxs = zeros(n, 1);
j = 1;
f = rand/n;
for i = 1:n
	c = c + w(i);
	while c > f
		idxs(j) = i;
		j = j + 1;
		f = f + 1/n;
	end
end
x = x(:, idxs)