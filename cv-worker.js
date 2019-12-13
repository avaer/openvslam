/* importScripts('/archae/plugins/_core_engines_resource/serve/three.js');
const {exports: THREE} = self.module;
importScripts('/archae/assets/murmurhash.js');
const {exports: murmur} = self.module;
importScripts('/archae/assets/autows.js');
const {exports: Autows} = self.module;
importScripts('/archae/assets/alea.js');
const {exports: alea} = self.module;
self.module = {}; */

// let Module = null;
// let slab = null;
/* const NUM_CELLS = 8;
const OVERSCAN = 1;
const NUM_CELLS_OVERSCAN = NUM_CELLS + OVERSCAN;
const NUM_CHUNKS_HEIGHT = 10;                 
const NUM_CELLS_HEIGHT = NUM_CELLS * NUM_CHUNKS_HEIGHT;
const NUM_CELLS_OVERSCAN_Y = NUM_CELLS_HEIGHT + OVERSCAN; */
/* const width = 10;
const height = 10;
const depth = 10; */
// let noiserOffset = 0;
self.Module = {
  onRuntimeInitialized() {
    // console.log('module loaded', self.Module);

    loaded = true;
    _flushMessages();
  },
};
importScripts('run_web.js');

class Allocator {
  constructor() {
    this.offsets = [];
  }
  alloc(constructor, size) {
    const offset = self.Module.__Z8doMallocm(size * constructor.BYTES_PER_ELEMENT);
    const b = new constructor(self.Module.HEAP8.buffer, self.Module.HEAP8.byteOffset + offset, size);
    b.offset = offset;
    this.offsets.push(offset);
    return b;
  }
  freeAll() {
    for (let i = 0; i < this.offsets.length; i++) {
      self.Module.__Z6doFreePv(this.offsets[i]);
    }
    this.offsets.length = 0;
  }
}

const queue = [];
let loaded = false;
const _handleMessage = data => {
  const {method} = data;
  switch (method) {
    case 'makeCalibrator': {
      const {width, height, type, boardWidth, boardHeight} = data;
      const calibratorPtr = self.Module.__Z15make_calibratoriiiii(width, height, type, boardWidth, boardHeight);
      const framebufPtr = self.Module.__Z23get_framebuf_calibratorP10Calibrator(calibratorPtr);

      self.postMessage({
        result: {
          calibratorPtr,
          framebufPtr,
        },
      });

      break;
    }
    case 'updateCalibrator': {
      const {calibratorPtr, framebufPtr, frameData} = data;
      // console.log('update calibrator', calibratorPtr, framebufPtr, frameData.length);
      for (let i3 = 0, i4 = 0; i4 < frameData.length; i3 += 3, i4 += 4) {
        self.Module.HEAPU8[framebufPtr+i3] = frameData[i4];
        self.Module.HEAPU8[framebufPtr+i3+1] = frameData[i4+1];
        self.Module.HEAPU8[framebufPtr+i3+2] = frameData[i4+2];
      }
      const ok = self.Module.__Z17update_calibratorP10Calibrator(calibratorPtr);

      self.postMessage({
        result: {
          ok,
        },
      });

      break;
    }
    case 'finishCalibrator': {
      const {calibratorPtr} = data;

      const allocator = new Allocator();

      const resultBuf = allocator.alloc(Uint8Array, 1024);
      const ok = self.Module.__Z17finish_calibratorP10CalibratorPh(calibratorPtr, resultBuf.offset);

      let index = 0;
      const cameraMatrix = new Float64Array(resultBuf.buffer, resultBuf.byteOffset + index, 3*3);
      index += 3*3*Float64Array.BYTES_PER_ELEMENT;
      const distCoeffs = new Float64Array(resultBuf.buffer, resultBuf.byteOffset + index, 1*8);
      index += 1*8*Float64Array.BYTES_PER_ELEMENT;

      self.postMessage({
        result: {
          ok,
          cameraMatrix,
          distCoeffs,
        },
      });

      allocator.freeAll();
      break;
    }
    /* case 'collide': {
      const allocator = new Allocator();

      const {positions: positionsData, indices: indicesData, origin: originData, direction: directionData} = data;

      const positions = allocator.alloc(Float32Array, positionsData.length);
      positions.set(positionsData);
      const indices = allocator.alloc(Uint32Array, indicesData.length);
      indices.set(indicesData);
      const origin = allocator.alloc(Float32Array, 3);
      origin[0] = originData[0];
      origin[1] = originData[1];
      origin[2] = originData[2];
      const direction = allocator.alloc(Float32Array, 3);
      direction[0] = directionData[0];
      direction[1] = directionData[1];
      direction[2] = directionData[2];
      const result = allocator.alloc(Float32Array, 3);

      self.LocalModule._doCollide(
        positions.offset,
        indices.offset,
        positions.length,
        indices.length,
        origin.offset,
        direction.offset,
        result.offset
      );

      self.postMessage({
        result: Float32Array.from([result[0], result[1], result[2]]),
      });

      allocator.freeAll();
      break;
    } */
    default: {
      console.warn('unknown method', data.method);
      break;
    }
  }
};
const _flushMessages = () => {
  for (let i = 0; i < queue.length; i++) {
    _handleMessage(queue[i]);
  }
};
self.onmessage = e => {
  const {data} = e;
  if (!loaded) {
    queue.push(data);
  } else {
    _handleMessage(data);
  }
};
