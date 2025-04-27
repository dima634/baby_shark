# Building for WebAssembly (WASM) with `wasm-pack`

Follow these steps to build the library to WebAssembly using `wasm-pack`:

1. **Install [wasm-pack](https://rustwasm.github.io/wasm-pack/installer/)**  
2. **Build the Library**  
    Use `wasm-pack` to build the library:
    ```bash
    wasm-pack build --target web
    ```
    This will generate WebAssembly and JavaScript bindings in the `pkg` directory.
3. **Using the Library**  
    You can now use the generated WebAssembly module in your JavaScript or TypeScript project. Initialize it like this:
    ```typescript
    import bs from 'baby_shark';

    let initialized = false;

    export async function initBabySharkAsync(): Promise<void> {
        if (initialized) {
            return;
        }

        const wasm = await fetch('baby_shark_bg.wasm');
        await bs(await wasm.arrayBuffer());

        initialized = true;
    }

    await initBabySharkAsync();
    ```
    Now you can use it in your JavaScript or TypeScript code. For example:
    ```typescript
    import * as bs from 'baby_shark';
    const vertices = new Float64Array([0, 0, 0, 1, 1, 1, 2, 2, 2]);
    const mesh = bs.Mesh.from_vertices(vertices);
    ```
