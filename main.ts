//% weight=100 color=#0fbc11 icon="\uf1eb"
namespace customlibrary {

    //% block
    export function sayHello(): void {
        basic.showString("Hello");
    }

    //% block
    export function addNumbers(a: number, b: number): number {
        return a + b;
    }
}
