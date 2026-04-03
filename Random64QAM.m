clc; clear; close all;

%% ===================== SERIAL SETUP =====================
biencom = "COM3";
sp = serialport(biencom, 115200, ...
    "DataBits",8,"Parity","none","StopBits",1,"FlowControl","none");
sp.Timeout = 10;
pause(2.0);
flush(sp, "input");

%% ===================== TX (32 bits) -> MCU =====================
dau  = input('Nhap chuoi 16 bit (vd: 010011...): ','s');
istr = regexprep(dau,'\s+','');
if length(istr) ~= 16
    error('Can nhap dung 16 bit.');
end
if any(istr ~= '0' & istr ~= '1')
    error('Chuoi chi duoc chua 0 va 1.');
end
tx2 = reshape(istr, 8, [])';
tx2 = uint8(bin2dec(tx2));
tx4 = uint8([0 0 tx2(1) tx2(2)]);
fprintf("TX 4 bytes (00 00 + data16): ");
fprintf("%02X ", tx4);
fprintf("\n");
flush(sp, "input");
write(sp, tx4, "uint8");
pause(0.05);
fprintf("Bytes available right after TX = %d\n", sp.NumBytesAvailable);

%% ===================== RX 32 bits <- MCU (4 bytes) =====================
t0 = tic;
while sp.NumBytesAvailable < 4
    if toc(t0) > sp.Timeout
        error('Timeout: khong nhan du 4 byte (32 bit) tu MCU.');
    end
    pause(0.001);
end
data4 = read(sp, 4, "uint8");
fprintf('Nhan du lieu 4 byte thanh cong.\n');
fprintf("RX 4 bytes: ");
fprintf("%02X ", data4);
fprintf("\n");

%% ===================== Convert 4 bytes -> 32-bit string =====================
num_bits = 32;
M = 64;
k = log2(M);
gray3  = {'000','001','011','010','110','111','101','100'};
levels = [-7 -5 -3 -1 1 3 5 7];
gray3_to_level = containers.Map(gray3, num2cell(levels));
level_to_gray3 = containers.Map(num2cell(levels), gray3);
Es_avg = 42;
normF  = sqrt(Es_avg);
in_str = '';
for i = 1:length(data4)
    in_str = [in_str, dec2bin(data4(i), 8)];
end
if length(in_str) ~= num_bits || any(in_str ~= '0' & in_str ~= '1')
    error('Loi: Chuoi nhan ve phai dung 32 bit va chi gom 0/1.');
end
tx_bits = in_str - '0';
%% ===================== 64-QAM mapping =====================
num_bits_pad = ceil(num_bits/k)*k;
if num_bits_pad > num_bits
    tx_bits_pad = [tx_bits, zeros(1, num_bits_pad-num_bits)];
else
    tx_bits_pad = tx_bits;
end

Nsym = num_bits_pad/k;
bit_groups = reshape(tx_bits_pad, k, []).';

symb = zeros(Nsym,1);
for i = 1:Nsym
    bits_str = sprintf('%d', bit_groups(i,:));
    I_bits = bits_str(1:3);
    Q_bits = bits_str(4:6);

    I = gray3_to_level(I_bits);
    Q = gray3_to_level(Q_bits);

    symb(i) = (I + 1j*Q) / normF;
end
target_err = 4;
accept_lo  = 4;
accept_hi  = 4;
EbN0_search = 0:1:30;
max_trials  = 12000;   
block_len   = 8;      
best.EbN0    = NaN;
best.err     = Inf;
best.rx_bits = [];
best.err_pos = [];
best.h       = [];
best.shat    = [];
bestWidth    = Inf;
bestScore    = Inf;
found = false;
for EbN0dB = EbN0_search
    for tr = 1:max_trials
        [rx_bits, err, err_pos, h, shat] = run_once_block_fading( ...
            EbN0dB, symb, Nsym, k, normF, levels, level_to_gray3, num_bits, tx_bits, block_len);

        if isempty(err_pos)
            width = Inf;
        else
            width = max(err_pos) - min(err_pos) + 1;
        end

        isBurst = is_burst_contiguous(err_pos, err);
        score = abs(err - target_err) + (isBurst==false)*5 + 0.1*max(0, width - err);

        if score < bestScore
            bestScore   = score;
            best.EbN0    = EbN0dB;
            best.err     = err;
            best.rx_bits = rx_bits;
            best.err_pos = err_pos;
            best.h       = h;
            best.shat    = shat;
            bestWidth    = width;
        end
        if (err >= accept_lo && err <= accept_hi) && isBurst
            found = true;
            break;
        end
    end
    if found, break; end
end

rx_str = char('0' + best.rx_bits);

fprintf('\n==============================\n');
fprintf('1-FRAME Rayleigh BLOCK fading + AWGN (burst errors)\n');
fprintf('block_len = %d symbols/block\n', block_len);
fprintf('Eb/N0 chon = %.1f dB\n', best.EbN0);
fprintf('TX 32 bits = %s\n', in_str);
fprintf('RX 32 bits = %s\n', rx_str);
fprintf('So bit sai = %d / 32\n', best.err);
if ~isempty(best.err_pos)
    fprintf('Vi tri sai (1-based) = ');
    fprintf('%d ', best.err_pos);
    fprintf('\n');
    fprintf('Do rong cum loi (width) = %d\n', bestWidth);
end
fprintf('Burst rule: EXACT 4 contiguous (vd 9,10,11,12)\n');
prefix_16_ones = repmat('1', 1, 16);
rx_str_with_prefix = [prefix_16_ones, char(best.rx_bits + '0')];
if length(rx_str_with_prefix) ~= 48
    error('Loi: Chuoi gui lai phai dung 48 bit.');
end
rx_bytes = reshape(rx_str_with_prefix, 8, [])';
rx_bytes = uint8(bin2dec(rx_bytes));
fprintf("TX 48-bit (binary) = %s\n", rx_str_with_prefix);
fprintf("TX 48-bit (hex 6 bytes) = ");
fprintf("%02X ", rx_bytes);
fprintf("\n");
flush(sp, "input");
write(sp, rx_bytes, "uint8");
disp('Da gui 6 byte (48 bit) qua UART.');
if sp.NumBytesAvailable > 0
    read(sp, sp.NumBytesAvailable, "uint8");
end
t1 = tic;
while sp.NumBytesAvailable < 6
    if toc(t1) > sp.Timeout
        warning('Khong nhan duoc 6 byte sau khi gui 48 bit.');
        break;
    end
    pause(0.001);
end
if sp.NumBytesAvailable >= 6
    rx6 = read(sp, 6, "uint8");
    last2 = rx6(end-1:end);
    fprintf("RX-after last 2 bytes: ");
    fprintf("%02X ", last2);
    fprintf("\n");
    rx16_after = [dec2bin(last2(1),8) dec2bin(last2(2),8)];
    fprintf("RX-after 16 bits: %s\n", rx16_after);
    if sp.NumBytesAvailable > 0
        read(sp, sp.NumBytesAvailable, "uint8");
    end
end
%% ===================== Plot =====================
subplot(2,1,1);
hold on; grid on; ylim([-0.2 1.2]);
tx_bites = istr - '0';  
stem(1:16, tx_bites, 'filled', 'LineWidth', 1.2); 
rx16_after_bits = rx16_after - '0';  
stem(1:16, rx16_after_bits, 'LineWidth', 1.2); 
xlabel('Vi tri bit'); ylabel('Gia tri');
title('So sánh 16 bit TX và 16 bit RX nhận được');
legend('16 bit Transfer','16 bit Receive','Location','best');
yticks([0 1]);
hold off;
bit_idx = 1:num_bits;
subplot(2,1,2);
hold on; grid on; ylim([-0.2 1.2]);
stem(bit_idx, tx_bits, 'filled', 'LineWidth', 1.2);
stem(bit_idx, best.rx_bits, 'LineWidth', 1.2);
if ~isempty(best.err_pos)
    plot(best.err_pos, tx_bits(best.err_pos), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
end
xlabel('Vi tri bit'); ylabel('Gia tri');
title(sprintf('TX vs RX | Eb/N0 = %.1f dB | err = %d | contiguous', best.EbN0, best.err));
legend('TX bits','RX bits','Vi tri sai','Location','best');
yticks([0 1]);
hold off;
function tf = is_burst_contiguous(err_pos, err)
    if isempty(err_pos) || err == 0
        tf = false; return;
    end
    if numel(err_pos) ~= err
        tf = false; return;
    end
    ep = sort(err_pos(:).');
    tf = (max(ep) - min(ep) + 1) == err;
end
function [rx_bits, err, err_pos, h, shat] = run_once_block_fading( ...
    EbN0dB, symb, Nsym, k, normF, levels, level_to_gray3, num_bits, tx_bits, block_len)
    h = zeros(Nsym,1);
    for b = 1:block_len:Nsym
        h_block = (randn + 1j*randn)/sqrt(2);
        idx = b:min(b+block_len-1, Nsym);
        h(idx) = h_block;
    end
    r_fade = h .* symb;
    EbN0 = 10^(EbN0dB/10);
    N0   = 1/(k*EbN0);
    noise = sqrt(N0/2)*(randn(Nsym,1)+1j*randn(Nsym,1));
    r     = r_fade + noise;
    shat = r ./ h;
    demod_I = zeros(Nsym,1);
    demod_Q = zeros(Nsym,1);
    for i = 1:Nsym
        I_rec = real(shat(i))*normF;
        Q_rec = imag(shat(i))*normF;

        [~, iI] = min(abs(levels - I_rec));
        [~, iQ] = min(abs(levels - Q_rec));

        demod_I(i) = levels(iI);
        demod_Q(i) = levels(iQ);
    end
    rx_bits_str = strings(1,Nsym);
    for i = 1:Nsym
        I_gray = level_to_gray3(demod_I(i));
        Q_gray = level_to_gray3(demod_Q(i));
        rx_bits_str(i) = string(I_gray) + string(Q_gray);
    end
    rx_bits_char = char(join(rx_bits_str,""));
    rx_bits_pad  = rx_bits_char - '0';
    rx_bits = rx_bits_pad(1:num_bits);
    err     = sum(tx_bits ~= rx_bits);
    err_pos = find(tx_bits ~= rx_bits);
end
