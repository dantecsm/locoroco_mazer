f=imread('x.jpg');    %f是黑白矩阵，白色表示可行路径
f=imresize(f,[100,100]);    %标准化为100*100尺寸
level=graythresh(f);    %二值化
f=im2bw(f,level);
%去除白色外边界
colsum=sum(f);
lr=find(colsum~=100);
left=lr(1);
right=lr(end);
rowsum=sum(f');
ud=find(rowsum~=100);
up=ud(1);
down=ud(end);
f=f([up:down],[left:right]);
%展示图像并取点，获取起点与终点
% f=~bwmorph(~f,'thin',inf);    %原图细化
figure,imshow(f);
set(gcf,'outerposition',get(0,'screensize'));
mnum=max(find(f==1));
point=ginput;
start=round(point(1,:));
goal=round(point(2,:));
%宽度优先遍历法寻找路径
vis=zeros(mnum);
fa=vis;
q=sub2ind(size(f),start(2),start(1));
root=q;
leaf=sub2ind(size(f),goal(2),goal(1));

while ~isempty(q)
    q1=q(1);
    [x y]=ind2sub(size(f),q1);
%     if [x y]==goal
%         break;
%     end
    for i=-1:1
        for j=-1:1
            if x+i<=0 | x+i>size(f,1) | y+j<=0 | y+j>size(f,2) | abs(i)==abs(j)
                continue;   %如果访问的下一点超出边界或是斜向的，跳过。
            end
            np=sub2ind(size(f),x+i,y+j);
            if f(x+i,y+j) & ~vis(np)
                q=[q np];
                vis(np)=1;
                fa(np)=q1;
            end
        end
    end
    q=q(2:end);
end
%维护路径数组
cur=leaf;
path=leaf;
ok=1;
while cur~=root
    if cur==0
        ok=0;
        break;
    end
    cur=fa(cur);
    path=[cur path];
end

if ok
    ok=1
    %根据路径数组与原图，生成答案图
    g1=zeros(size(f));
    g1(f==1)=255;
%     g1(path)=0;
    g2=zeros(size(f));
    g2(f==1)=255;
%     g2(path)=0;
    g3=zeros(size(f));
    g3(f==1)=255;
%     g3(path)=0;
    
leng=floor(length(path)/10);
    for i=1:leng+1
        if i*10<=length(path)
            way{i}=path((i-1)*10+1:i*10);
        else
            way{i}=path((i-1)*10+1:length(path));
        end
        g1(way{i})=0;
        g2(way{i})=0;
        g3(way{i})=255;
        g=cat(3,g1,g2,g3);
        g=imresize(g,[200,200],'nearest');
        frame{i}=g;
        figure,imshow(frame{i});
        set(gcf,'outerposition',get(0,'screensize'));
        im=frame2im(getframe(gcf));
        [l,map]=rgb2ind(im,20);
        if i==1
            imwrite(l,map,'solution.gif','gif','Loopcount',400,'DelayTime',0.1);
        else
            imwrite(l,map,'solution.gif','gif','WriteMode','append','DelayTime',0.1);
        end
        close all
        filename=['pic' num2str(i) '.png'];
        imwrite(frame{i},filename);
    end
    
    g=cat(3,g1,g2,g3);
    figure,imshow(g);
    set(gcf,'outerposition',get(0,'screensize'));
else
    ok=0
end