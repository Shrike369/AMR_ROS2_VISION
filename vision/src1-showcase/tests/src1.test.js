const { myFunction } = require('../src1/index');

describe('src1 Project Tests', () => {
    test('myFunction should return expected result', () => {
        const input = 'test input';
        const expectedOutput = 'expected output';
        expect(myFunction(input)).toBe(expectedOutput);
    });

    // Add more tests as needed
});