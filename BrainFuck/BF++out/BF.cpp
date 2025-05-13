// 定义了一个简单的BrainFuck程序代码，输出"Hello World!"
#define BFCODE \
"++++++++[>++++[>++>+++>+++>+<<<<-]>+>+>->>+[<]<-]>>.>---.+++++++..+++.>>.<-.<.+++.------.--------.>>+.>++."

// 编译命令示例
// PS > g++ .\BF.cpp -o BF

//////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <type_traits>

// 流式输出缓冲区模板类
// N: 缓冲区大小
template <size_t N>
class Stream
{
public:
    // 向缓冲区添加一个字符
    constexpr void push(char c) { data_[idx_++] = c; }
    // 转换为C风格字符串
    constexpr operator const char *() const { return data_; }
    // 获取当前缓冲区大小
    constexpr size_t size() { return idx_; }

private:
    size_t idx_{};      // 当前写入位置索引
    char data_[N]{};    // 字符缓冲区
};

// BrainFuck解析函数(递归实现)
// input: BrainFuck代码字符串
// skip: 是否跳过执行(用于处理循环)
// cells: 内存单元数组
// pc: 当前指针位置
// output: 输出流
template <typename STREAM>
constexpr auto parse(const char *input, bool skip, char *cells,
                     size_t &pc, STREAM &&output) -> size_t
{
    const char *c = input;
    while (*c)
    {
        switch (*c)
        {
        case '+': // 当前内存单元值加1
            if (!skip)
                ++cells[pc];
            break;
        case '-': // 当前内存单元值减1
            if (!skip)
                --cells[pc];
            break;
        case '.': // 输出当前内存单元值对应的ASCII字符
            if (!skip)
                output.push(cells[pc]);
            break;
        case '>': // 指针右移
            if (!skip)
                ++pc;
            break;
        case '<': // 指针左移
            if (!skip)
                --pc;
            break;
        case '[': // 循环开始
        {
            // 当当前内存单元值不为0时执行循环体
            while (!skip && cells[pc] != 0)
                parse(c + 1, false, cells, pc, output);
            // 跳过循环体(用于处理嵌套循环)
            c += parse(c + 1, true, cells, pc, output) + 1;
        }
        break;
        case ']': // 循环结束
            return c - input; // 返回已解析的字符数
        default:  // 忽略其他字符
            break;
        }
        ++c;
    }
    return c - input; // 返回已解析的字符数
}

// 内存单元大小
constexpr size_t CELL_SIZE = 16;

// BrainFuck解析函数(外部接口)
// input: BrainFuck代码字符串
// output: 输出流
template <typename STREAM>
constexpr auto parse(const char *input, STREAM &&output) -> STREAM &&
{
    char cells[CELL_SIZE]{}; // 初始化内存单元
    size_t pc{};            // 初始化指针位置
    parse(input, false, cells, pc, output); // 开始解析
    return output;
}

// BrainFuck执行函数(模板版本)
// OUTPUT_SIZE: 输出缓冲区大小
// input: BrainFuck代码字符串
template <size_t OUTPUT_SIZE = 15>
constexpr auto brain_fuck(const char *input)
{
    // 创建一个 Stream 类型的对象 output，其大小为 OUTPUT_SIZE
    // Stream 应该是一个自定义的类，用于处理输出
    Stream<OUTPUT_SIZE> output;
    // 调用 parse 函数，传入输入字符串 input 和输出对象 output
    // parse 函数应该是一个自定义的函数，用于解析 Brainfuck 代码
    return parse(input, output);
}
// 定义一个函数 brain_fuck_output_size，用于计算 Brainfuck 代码的输出大小
// 接受一个指向常量字符的指针 input 作为输入
constexpr auto brain_fuck_output_size(const char *input) -> size_t
{
    struct
    {
        size_t sz{}; // 初始化为 0
        // 定义一个函数 push，接受任意数量的参数
        // 每次调用 push 函数时，sz 的值加 1
        constexpr void push(...) { ++sz; }
    } dummy;
    // 调用 parse 函数，传入输入字符串 input 和模拟输出对象 dummy
    // 并返回模拟输出对象的 sz 值加 1
    return parse(input, dummy).sz + 1;
}
// 定义一个宏 BRAIN_FUCK，用于简化调用 brain_fuck 函数
// 该宏接受一个参数 in，将其作为 Brainfuck 代码
// 调用 brain_fuck 函数时，使用 brain_fuck_output_size 函数计算输出大小
#define BRAIN_FUCK(in) brain_fuck<brain_fuck_output_size(in)>(in)
// 主函数，程序的入口点
int main()
{
    // 调用 puts 函数，输出 BRAIN_FUCK 宏处理后的结果
    // BFCODE 应该是一个预定义的 Brainfuck 代码字符串
    puts(BRAIN_FUCK(BFCODE));
}
