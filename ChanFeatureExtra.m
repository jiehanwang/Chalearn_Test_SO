function ChanFeatureExtra(samplePath, meaning_ID)

    % samplePath = 'Sample00001.zip';
    mean_ID=cell(1,20);
    
    % Split the path and the filename
    [path,sampleID,ext]=fileparts(samplePath);
    FileName_long = ['output\' sampleID '.txt'];
    fid = fopen(FileName_long,'wt');

    % Check that the given file is a zip file
    if strcmp(ext,'.zip')~=1,
        warning('Expected a ZIP file as input');
    end

    % Check if there is a folder for this sample (unziped) or we need to unzip the file.
    sampleDir=fullfile(path,sampleID);
    src='zip';
    if exist(sampleDir,'dir'),
        src='dir';
    end

    % If it is necessary, unzip the file
    if strcmp(src,'zip'),
        try
           files=unzip(samplePath,sampleDir);
           if isempty(files),
               warning(['Empty extracted file: ' samplePath]);
           end
        catch
            warning(['Cannot extract file: ' samplePath]);
        end

    end

    % Get the video information
    videoData=load(fullfile(sampleDir,[sampleID '_data.mat']));
    videoData=videoData.Video;
    videoData.SampleID=sampleID;

     % Get the labels
     if nargout>1,
         % Get frame labels
         labels=getLabels(videoData);

         % Get list of gestures
         labels=labels(labels~=[0 labels(1:end-1)]);

         % Remove the zeros
         labels=labels(labels~=0);
     end
     
     % Get the Skeleton data
    for i = 1:videoData.NumFrames
       videoData.Skeleton(i)=videoData.Frames(i).Skeleton;
    end
    videoData = rmfield(videoData,'Frames');   % remove field from structure
     
     
     FileName_color = fullfile(sampleDir,[sampleID '_color.mp4']);
     FileName_depth = fullfile(sampleDir,[sampleID '_depth.mp4']);
     FileName_seg   = fullfile(sampleDir,[sampleID '_user.mp4']);
     videoFile_color = vision.VideoFileReader(FileName_color);
     videoFile_depth = vision.VideoFileReader(FileName_depth);
     videoFile_seg = vision.VideoFileReader(FileName_seg);
     
     % Initialization and settings
     length = 60;
     handRegion_color = zeros(length,length*2,3);
     handRegion_depth = zeros(length,length*2);
     fprintf(fid,'%d %d\n',videoData.NumFrames,658);
     Feature_all = zeros(videoData.NumFrames,658);
     h_mean = 0;
     v_mean = 0;

     for i = 1:videoData.NumFrames
            fprintf('%s %d / %d \n',samplePath, i, videoData.NumFrames);
            
            % 得到color，depth信息
            frames_color = step(videoFile_color);
            frames_depth = getDepth(step(videoFile_depth),videoData.MaxDepth);
            frames_seg   = rgb2gray(step(videoFile_seg));
            
            % 用seg信息去除背景
%             frames_seg(frames_seg>0)=1;
%             for cha = 1:3
%                 frames_color(:,:,cha) = frames_color(:,:,cha).*frames_seg;
%             end
%             frames_depth = frames_depth.*frames_seg;
            
            % skin model from face 
            % skin model definition
            head_size = 15;
            head_col = videoData.Skeleton(i).PixelPosition(4,2);
            head_row = videoData.Skeleton(i).PixelPosition(4,1);
%             head_x1 = max(head_col-head_size, 1);
%             head_x2 = min(head_col+head_size, 640);
%             head_y1 = max(head_row-head_size, 1);
%             head_y2 = min(head_row+head_size, 480);
%             head_HSI = rgb2hsi(frames_color(head_x1:head_x2,head_y1:head_y2,:));
%             h_mean = mean(mean(head_HSI(:,:,1)));%0.2203; %mean(mean(head_HSI(:,:,1)));
%             s_mean = mean(mean(head_HSI(:,:,2)));%0.2613; %mean(mean(head_HSI(:,:,2)));
%             v_mean = mean(mean(head_HSI(:,:,3)));%0.4281; %mean(mean(head_HSI(:,:,3)));
%             fprintf('%d %d %d',h_mean, s_mean, v_mean);
            
            
            % Remove the non-skin region
%             frames_color_HSI = rgb2hsi(frames_color);
%             for row=1:480
%                 for col=1:640
%                     if frames_seg(row, col) == 1
%                         if (~((frames_color_HSI(row, col,2)-s_mean)^2< 0.1 ...
%                                 && (frames_color_HSI(row, col,3)-v_mean)^2< 0.1...
%                                 && (frames_color_HSI(row, col,1)-h_mean)^2< 0.05))
%                             frames_color(row,col,:) = 0;
%                         end
%                     end
%                 end
%             end
            
            
            % 确定左右手的bounding box
            %  --------x----------
            % |
            % y
            % |
            leftHand_row = videoData.Skeleton(i).PixelPosition(8,1); % 左手中心所在行 y
            leftHand_col = videoData.Skeleton(i).PixelPosition(8,2); % 左手中心所在列 x
            left_x1 = max(leftHand_col-length/2+1, 1);
            left_x2 = min(leftHand_col+length/2,640);
            left_y1 = max(leftHand_row-length/2+1, 1);
            left_y2 = min(leftHand_row+length/2,480);
            
            rightHand_row = videoData.Skeleton(i).PixelPosition(12,1);% 右手中心所在行 y
            rightHand_col = videoData.Skeleton(i).PixelPosition(12,2);% 右手中心所在列 x
            right_x1 = max(rightHand_col-length/2+1, 1);
            right_x2 = min(rightHand_col+length/2, 640);
            right_y1 = max(rightHand_row-length/2+1, 1);
            right_y2 = min(rightHand_row+length/2, 480);
            
            %左右手抠图
            handRegion_color_right = frames_color(right_x1:right_x2, right_y1:right_y2,:);
            handRegion_depth_right = frames_depth(right_x1:right_x2,right_y1:right_y2,:);
            handRegion_color_left = frames_color(left_x1:left_x2,left_y1:left_y2, :);
            handRegion_depth_left = frames_depth(left_x1:left_x2,left_y1:left_y2, :);
            

            % 将左右手合并成一张图
            if (size(handRegion_color_right(:,:,1),2)<length ||...   %防止没有检测到，或者检测到图像边缘的情况发生
                    size(handRegion_color_left(:,:,1),2)<length ||...
                    size(handRegion_color_right(:,:,1),1)<length ||...
                    size(handRegion_color_left(:,:,1),1)<length)
                handRegion_color(1:length,1:length,:) = 0;
                handRegion_color(1:length,length+1:2*length,:) = 0;
                handRegion_depth(1:length,1:length,:) = 0;
                handRegion_depth(1:length,length+1:2*length,:) = 0;
            else
                handRegion_color(1:length,1:length,:) = handRegion_color_right;
                handRegion_color(1:length,length+1:2*length,:) = handRegion_color_left;
                
                handRegion_depth(1:length,1:length,:) = handRegion_depth_right;
                handRegion_depth(1:length,length+1:2*length,:) = handRegion_depth_left;
            end
            
            % For saving
            hand_image{i} = handRegion_color;
            left_height(i) = leftHand_col;  % 此处居然要用col,但是经测试，是对的。
            right_height(i) = rightHand_col;
%             fprintf('left_height: %f\n', leftHand_col);  
%             fprintf('right_height: %f\n', rightHand_col);
            
            
            %左右手速度
            if i>5
                left_speed(i) = (videoData.Skeleton(i).WorldPosition(8,1)-videoData.Skeleton(i-1).WorldPosition(8,1))^2+...
                        (videoData.Skeleton(i).WorldPosition(8,2)-videoData.Skeleton(i-1).WorldPosition(8,2))^2+...
                        (videoData.Skeleton(i).WorldPosition(8,3)-videoData.Skeleton(i-1).WorldPosition(8,3))^2;
                right_speed(i) = (videoData.Skeleton(i).WorldPosition(12,1)-videoData.Skeleton(i-1).WorldPosition(12,1))^2+...
                    (videoData.Skeleton(i).WorldPosition(12,2)-videoData.Skeleton(i-1).WorldPosition(12,2))^2+...
                    (videoData.Skeleton(i).WorldPosition(12,3)-videoData.Skeleton(i-1).WorldPosition(12,3))^2;
            else
                left_speed(i) = 0;
                right_speed(i) = 0;
            end

          
            % 求HOG手型特征
            % 左右手合在一张图上
%             handRegion_color_gray = rgb2gray(handRegion_color);
%             H = HOG(handRegion_color_gray);
%             H = HOG(handRegion_depth);
%             [VSFAT Threshold]=edge(grayPic,'sobel','vertical'); 
%             Posture_dim = 324;
            %左右手分别求
            handRegion_color_left_gray = rgb2gray(handRegion_color_left);
            handRegion_color_right_gray = rgb2gray(handRegion_color_right);
            H1 = HOG(handRegion_color_left_gray);
            H2 = HOG(handRegion_color_right_gray);
%             H1 = HOG(handRegion_depth_left);
%             H2 = HOG(handRegion_depth_right);
            H(1:size(H1)) = H1;
            H(size(H1)+1:2*size(H1))=H2;
            Posture_dim = 648;
            
            
            % 求SP骨架特征
            sp = zeros(1,10);
            sp_count = 1;
            for sk=4:2:10
                for sk2=sk+2:2:12
                    sp(sp_count) = (videoData.Skeleton(i).WorldPosition(sk,1)-videoData.Skeleton(i).WorldPosition(sk2,1))^2+...
                        (videoData.Skeleton(i).WorldPosition(sk,2)-videoData.Skeleton(i).WorldPosition(sk2,2))^2+...
                        (videoData.Skeleton(i).WorldPosition(sk,3)-videoData.Skeleton(i).WorldPosition(sk2,3))^2;
                    sp_count = sp_count + 1;
                end
            end
            sp = sp/max(sp);
            SP_dim = 10;
                          
            % 连接两个特征，并且输出整个视频的特征文件
            for f=1:Posture_dim+SP_dim
                if f<11
                    if isnan(sp(f))
                        fprintf(fid,'%f ', 0);
                        Feature_all(i,f) = 0;
                    else
                        fprintf(fid,'%f ', sp(f));
                        Feature_all(i,f) = sp(f);
                    end
                else
                    if isnan(H(f-10))
                        fprintf(fid,'%f ', 0);
                        Feature_all(i,f) = 0;
                    else
                        fprintf(fid, '%f ', H(f-10));
                        Feature_all(i,f) = H(f-10);
                    end
                end
            end
            fprintf(fid, '\n', H(f-10));
            

            % 连通域
%             thre = 0.3;
%             handRegion_color_gray(handRegion_color_gray<thre) = 0;
%             handRegion_color_gray(handRegion_color_gray>=thre) = 1;
%             handRegion_color_gray = bwareaopen(handRegion_color_gray,20);
            
            % Show the image
%             imshow(frames_color);  % frames_color, handRegion_color, handRegion_depth
%             %rectangle中的xy和矩阵中的行列不一样
%             rectangle('Position',[head_row-head_size, head_col-head_size, head_size*2, head_size*2 ], 'EdgeColor','r'); 
%             rectangle('Position',[right_y1, right_x1,  length, length], 'EdgeColor','r');
%             rectangle('Position',[left_y1, left_x1, length, length], 'EdgeColor','r');
%             drawnow;
     end
     fclose(fid);

     
     % 记录分词特征
     items = size(videoData.Labels, 2);
     for s=1:items
         sign_name = videoData.Labels(1,s).Name;
         sign_ID = 0;
         % 找出对应单词的ID
         for k=1:20
             if strcmp(sign_name, meaning_ID{k,1})
                 sign_ID = k;
                 break;
             end
         end
         % 找到开始和结束
         startF = videoData.Labels(1,s).Begin;
         endF = videoData.Labels(1,s).End;
         nframes = endF-startF+1;
         if sign_ID<10
             FileName_sign = ['output\0' num2str(sign_ID) '\0' num2str(sign_ID) '_' sampleID '.txt'];
         else
             FileName_sign = ['output\' num2str(sign_ID) '\' num2str(sign_ID) '_' sampleID '.txt'];
         end
         
         
         
         % 获得key frame 的数值
         keyThre = 100; %0.0005;
         nKeyframes = 0;
         for f=1:nframes
             if ((left_speed(startF+f-1) < keyThre && right_speed(startF+f-1) < keyThre) &&...
                     ( (left_height(startF+f-1) < 300 || right_height(startF+f-1)<300) && sign_ID~=5 ))
%              if ((left_speed(startF+f-1) < keyThre && right_speed(startF+f-1) < keyThre))
                 nKeyframes = nKeyframes+1;
             end
         end
         % 记录txt
         fid_sign = fopen(FileName_sign, 'wt');
         fprintf(fid_sign, '%d %d\n',nKeyframes,(Posture_dim+SP_dim));
         for f=1:nframes
             % 对低速的frame提取手型特征
             if ((left_speed(startF+f-1) < keyThre && right_speed(startF+f-1) < keyThre) &&...
                     ( (left_height(startF+f-1) < 300 || right_height(startF+f-1)<300) && sign_ID~=5 ))
%              if ((left_speed(startF+f-1) < keyThre && right_speed(startF+f-1) < keyThre))
                 % 保存图片
                 if sign_ID<10
                     FileName_sign_folder = ['output\0' num2str(sign_ID) '\0' num2str(sign_ID) '_' sampleID];
                     FileName_sign_pic = ['output\0' num2str(sign_ID) '\0' num2str(sign_ID) '_' sampleID '\' num2str(f) '.jpg'];
                 else
                     FileName_sign_folder = ['output\' num2str(sign_ID) '\' num2str(sign_ID) '_' sampleID];
                     FileName_sign_pic = ['output\' num2str(sign_ID) '\' num2str(sign_ID) '_' sampleID '\' num2str(f) '.jpg'];
                 end
                 mkdir(FileName_sign_folder);
                 imwrite(hand_image{startF+f-1}, FileName_sign_pic);
                 %记录特征
                 for fea = 1:(Posture_dim+SP_dim)
                     fprintf(fid_sign, '%f ', Feature_all(startF+f-1, fea));
                 end
                 fprintf(fid_sign, '\n');
             end
             
         end
         fclose(fid_sign);
     end
     

     
     % If we unziped the file, remove the folder
    if strcmp(src,'zip'),
        recycleStat=recycle('off');
        try
            rmdir(sampleDir,'s');        
        catch err
            warning(['Cannot remove foler: ' sampleDir ' error: ' err.message]);
        end
       
        recycle(recycleStat);
    end
end
 
 %%
 function labels=getLabels(video)
    % Define an initial label for each frame (0-> no gesture)
    labels=zeros(1,video.NumFrames);
    
    % Initialize the stats
    stats=zeros(1,20);
        
    % For each gesture in the sample generate corresponding labels
    for lab=video.Labels,
        error=0;
        % Check the labels 
        if lab.Begin<=0 || lab.End<=0,
            if error==0,
                error=1;
                warning(['Null index errors in sample: (Gesture -> ' lab.Name ')']);
            end
            continue;
        end
        % Check that the index are correct
        if lab.Begin<1 || lab.End<1 || lab.End<=lab.Begin || lab.End>video.NumFrames,
            warning(['Index error in sample:  (Gesture -> ' lab.Name ')']);
            tmp=lab.Begin;
            lab.Begin=lab.End;
            lab.End=tmp;
        end
        % Check that all afected frames are 0
        if sum(labels(lab.Begin:lab.End))~=0,
            warning('There are overlapped gestures in sample.');
        end
        
        % Get the gesture identifier        
        id=getGestureID(lab.Name);
        
        % Add this value
        stats(id)=stats(id)+1;
                
        % Check that the gesture name is correct
        if id<=0,
            warning(['Unrecognized gesture (' lab.Name ')']);
        end
        
        % Set the labels
        labels(lab.Begin:lab.End)=id;  
    end
 end

 function depth = getDepth(frame,maxDepth)
%     depth = uint16(round(double(rgb2gray(frame))/255.0*maxDepth));
    depth = ((double(rgb2gray(frame))));
 end

 
 function im = skinSeg(I)
     I=double(I); 
    [hue,s,v]=rgb2hsv(I); 

    cb = 0.148* I(:,:,1) - 0.291* I(:,:,2) + 0.439 * I(:,:,3) + 128; 
    cr = 0.439 * I(:,:,1) - 0.368 * I(:,:,2) -0.071 * I(:,:,3) + 128; 
    [w h]=size(I(:,:,1)); 

    for i=1:w 
     for j=1:h 
     if 140<=cr(i,j) && cr(i,j)<=165 && 140<=cb(i,j) && cb(i,j)<=195 && 0.01<=hue(i,j) && hue(i,j)<=0.1 
     segment(i,j)=1; 
     else 
     segment(i,j)=0; 
     end 
     end 
    end 

    imshow(segment); 
    im(:,:,1)=I(:,:,1).*segment; 
    im(:,:,2)=I(:,:,2).*segment; 
    im(:,:,3)=I(:,:,3).*segment; 
 end
 
 function hsi=rgb2hsi(rgb)
%RGB2HSI Converts an RGB image to HSI
%   HSI=RGB2HSI(rgb) converts an RGB image to HSI. The input image is
%   assumed to be of size M-by-N-by-3, where the third dimension accounts
%   for three image planes:red, green, and blue, in that order. If all RGB
%   component images are equal, the HSI conversion is undefined. Ths input
%   image can be of class double (with values in the rang[0,1]), uint8, or
%   uint16.
%   The output image, HSI, is of class double, where:
%       hsi(:,:,1)= hue image normalized values to the range [0,1] by
%                   dividing all angle values by 2*pi.
%       hsi(:,:,2)=saturation image, in the range [0,1].
%       hsi(:,:,3)=intensity image, in the range [0,1].
%Extract the individual component images.
rgb=im2double(rgb);
r=rgb(:,:,1);
g=rgb(:,:,2);
b=rgb(:,:,3);
%Implement the conversion equations.
num=0.5*((r-g)+(r-b));
den=sqrt((r-g).^2+(r-b).*(g-b));
theta=acos(num./(den+eps));
H=theta;
H(b>g)=2*pi-H(b>g);
H=H/(2*pi);

num=min(min(r,g),b);
den=r+g+b;
den(den==0)=eps;
S=1-3.*num./den;
H(S==0)=0;
I=(r+g+b)/3;
%Combine all three results into an hsi image.
hsi=cat(3,H,S,I);
 end
